/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-18

  (C) Copyright 2015-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_crtk/mtsROSToCISST.h>

#include <dvrk_utilities/dvrk_console.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisst_ros_crtk/cisst_ros_crtk.h>

#include <cisstCommon/cmnStrings.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocus.h>

#include <saw_robot_io_1394_ros/mts_ros_crtk_robot_io_bridge.h>
#include <saw_controllers_ros/mts_ros_crtk_controllers_pid_bridge.h>

#include <json/json.h>

const std::string bridgeNamePrefix = "dVRKIOBridge_";

CMN_IMPLEMENT_SERVICES(dvrk_console);

dvrk::console::console(const std::string & name,
                       cisst_ral::node_ptr_t node_handle,
                       const double & publish_rate_in_seconds,
                       const double & tf_rate_in_seconds,
                       mtsIntuitiveResearchKitConsole * mts_console):
    mts_ros_crtk_bridge_provided(name, node_handle),
    m_publish_rate(publish_rate_in_seconds),
    m_tf_rate(tf_rate_in_seconds),
    m_console(mts_console)
{
    // start creating components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create all ROS bridges
    std::string m_bridge_name = "dvrk_ros" + cisst_ral::node_namespace(node_handle);
    cisst_ral::clean_namespace(m_bridge_name);

    // shared publish bridge
    m_pub_bridge = new mtsROSBridge(m_bridge_name, publish_rate_in_seconds, node_handle);
    m_pub_bridge->AddIntervalStatisticsInterface();
    componentManager->AddComponent(m_pub_bridge);

    // get the stats bridge from base class
    stats_bridge().AddIntervalStatisticsPublisher("publishers", m_pub_bridge->GetName());

    // IO topics
    if (m_console->mHasIO) {
        add_topics_io();
    }

    // arm topics
    for (const auto & armPair : m_console->mArms) {
        auto arm = armPair.second;
        if (!arm->m_skip_ROS_bridge) {
            const std::string name = armPair.first;
            if (arm->native_or_derived_mtm()) {
                bridge_interface_provided_mtm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
            } else if (arm->native_or_derived_ecm()) {
                bridge_interface_provided_ecm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                if (arm->m_simulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    add_topics_ecm_io(name, arm->m_IO_component_name);
                }
            } else if (arm->native_or_derived_psm()) {
                bridge_interface_provided_psm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                if (arm->m_simulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    add_topics_psm_io(name, arm->m_IO_component_name);
                }
            } else if (arm->generic()) {
                bridge_interface_provided(arm->ComponentName(),
                                          arm->InterfaceName(),
                                          publish_rate_in_seconds, tf_rate_in_seconds);
            } else if (arm->suj()) {
                const auto _sujs = std::list<std::string>({"PSM1", "PSM2", "PSM3", "ECM"});
                for (auto const & _suj : _sujs) {
                    bridge_interface_provided(name,
                                              _suj,
                                              "SUJ/" + _suj,
                                              publish_rate_in_seconds,
                                              tf_rate_in_seconds);
                }
            }
        }
    }

    // ECM teleop
    if (m_console->mTeleopECM) {
        const auto teleopName = m_console->mTeleopECM->Name();
        bridge_interface_provided(teleopName, "Setting", teleopName,
                                  publish_rate_in_seconds, 0.0); // do no republish info already provided by arm, set tf period to 0
        add_topics_teleop_ecm(teleopName);
    }

    // PSM teleops
    for (auto const & teleop : m_console->mTeleopsPSM) {
        const auto teleopName = teleop.first;
        bridge_interface_provided(teleopName, "Setting", teleopName,
                                  publish_rate_in_seconds, 0.0); // do no republish info already provided by arm, set tf period to 0
        add_topics_teleop_psm(teleopName);
    }

    // Endoscope focus
    if (m_console->mDaVinciEndoscopeFocus) {
        add_topics_endoscope_focus();
    }

    // digital inputs
    const std::string footPedalsNameSpace = "footpedals/";
    for (auto input : m_console->mDInputSources) {
        std::string upperName = input.second.second;
        std::string lowerName = input.first;
        std::string requiredInterfaceName = upperName + "_" + lowerName;
        // put everything lower case
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(), tolower);
        // replace +/- by strings
        cmnStringReplaceAll(lowerName, "-", "_minus");
        cmnStringReplaceAll(lowerName, "+", "_plus");
        events_bridge().AddPublisherFromEventWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
            (requiredInterfaceName, "Button",
             footPedalsNameSpace + lowerName);
        m_connections.Add(events_bridge().GetName(), requiredInterfaceName,
                          input.second.first, input.second.second);

    }

    // console
    add_topics_console();
}

void dvrk::console::bridge_interface_provided_arm(const std::string & _arm_name,
                                                  const std::string & _interface_name,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all CRTK topics
    bridge_interface_provided(_arm_name, _interface_name, _arm_name,
                              _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK arm

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _arm_name + "_using_" + _interface_name;

    subscribers_bridge().AddSubscriberToCommandWrite<prmPositionCartesianSet, CISST_RAL_MSG(geometry_msgs, PoseStamped)>
        (_required_interface_name, "set_base_frame",
         _arm_name + "/set_base_frame");
    subscribers_bridge().AddSubscriberToCommandWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_required_interface_name, "trajectory_j/set_ratio",
         _arm_name + "/trajectory_j/set_ratio");
    subscribers_bridge().AddSubscriberToCommandWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_required_interface_name, "trajectory_j/set_ratio_v",
         _arm_name + "/trajectory_j/set_ratio_v");
    subscribers_bridge().AddSubscriberToCommandWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_required_interface_name, "trajectory_j/set_ratio_a",
         _arm_name + "/trajectory_j/set_ratio_a");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_required_interface_name, "body/set_cf_orientation_absolute",
         _arm_name + "/body/set_cf_orientation_absolute");
    subscribers_bridge().AddServiceFromCommandQualifiedRead<vctDoubleVec, vctDoubleVec, CISST_RAL_SRV(cisst_msgs, ConvertFloat64Array)>
        (_required_interface_name, "actuator_to_joint_position",
         _arm_name + "/actuator_to_joint_position");

    events_bridge().AddPublisherFromEventWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_required_interface_name, "desired_state",
         _arm_name + "/desired_state");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_required_interface_name, "goal_reached",
         _arm_name + "/goal_reached");
    events_bridge().AddPublisherFromEventWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_required_interface_name, "trajectory_j/ratio",
         _arm_name + "/trajectory_j/ratio");
    events_bridge().AddPublisherFromEventWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_required_interface_name, "trajectory_j/ratio_v",
         _arm_name + "/trajectory_j/ratio_v");
    events_bridge().AddPublisherFromEventWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_required_interface_name, "trajectory_j/ratio_a",
         _arm_name + "/trajectory_j/ratio_a");
}

void dvrk::console::bridge_interface_provided_ecm(const std::string & _arm_name,
                                                  const std::string & _interface_name,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_arm_name, _interface_name,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK ECM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _arm_name + "_using_" + _interface_name;

    subscribers_bridge().AddSubscriberToCommandWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_required_interface_name, "set_endoscope_type",
         _arm_name + "/set_endoscope_type");

    events_bridge().AddPublisherFromEventWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_required_interface_name, "endoscope_type",
         _arm_name + "/endoscope_type");
    events_bridge().AddPublisherFromEventWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
        (_required_interface_name, "ManipClutch",
         _arm_name + "/manip_clutch");
}

void dvrk::console::bridge_interface_provided_mtm(const std::string & _arm_name,
                                                  const std::string & _interface_name,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_arm_name, _interface_name,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK MTM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _arm_name + "_using_" + _interface_name;

    subscribers_bridge().AddSubscriberToCommandWrite<vctMatRot3, CISST_RAL_MSG(geometry_msgs, Quaternion)>
        (_required_interface_name, "lock_orientation",
         _arm_name + "/lock_orientation");
    subscribers_bridge().AddSubscriberToCommandVoid
        (_required_interface_name, "unlock_orientation",
         _arm_name + "/unlock_orientation");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_required_interface_name, "orientation_locked",
         _arm_name + "/orientation_locked");

    events_bridge().AddPublisherFromEventVoid
        (_required_interface_name, "gripper/pinch",
         _arm_name + "/gripper/pinch");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_required_interface_name, "gripper/closed",
         _arm_name + "/gripper/closed");
}

void dvrk::console::bridge_interface_provided_psm(const std::string & _arm_name,
                                                  const std::string & _interface_name,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_arm_name, _interface_name,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK PSM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _arm_name + "_using_" + _interface_name;

    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_required_interface_name, "emulate_adapter_present",
         _arm_name + "/emulate_adapter_present");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_required_interface_name, "emulate_tool_present",
         _arm_name + "/emulate_tool_present");
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_required_interface_name, "set_tool_type",
         _arm_name + "/set_tool_type");

    events_bridge().AddPublisherFromEventWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
        (_required_interface_name, "ManipClutch",
         _arm_name + "/manip_clutch");
    events_bridge().AddPublisherFromEventVoid
        (_required_interface_name, "tool_type_request",
         _arm_name + "/tool_type_request");
    events_bridge().AddPublisherFromEventWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_required_interface_name, "tool_type",
         _arm_name + "/tool_type");
}

void dvrk::console::add_topics_console(void)
{
    const std::string _ros_namespace = "console/";

    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "power_off",
         _ros_namespace + "power_off");
    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "power_on",
         _ros_namespace + "power_on");
    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "home",
         _ros_namespace + "home");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        ("Console", "teleop_enable",
         _ros_namespace + "teleop/enable");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        ("Console", "teleop_enabled",
         _ros_namespace + "teleop/enabled");

    subscribers_bridge().AddSubscriberToCommandWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        ("Console", "cycle_teleop_psm_by_mtm",
         _ros_namespace + "teleop/cycle_teleop_psm_by_mtm");
    subscribers_bridge().AddSubscriberToCommandWrite<prmKeyValue, CISST_RAL_MSG(diagnostic_msgs, KeyValue)>
        ("Console", "select_teleop_psm",
         _ros_namespace + "teleop/select_teleop_psm");
    subscribers_bridge().AddSubscriberToCommandWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        ("Console", "set_scale",
         _ros_namespace + "teleop/set_scale");

    events_bridge().AddPublisherFromEventWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        ("Console", "scale",
         _ros_namespace + "teleop/scale");
    events_bridge().AddPublisherFromEventWrite<prmKeyValue, CISST_RAL_MSG(diagnostic_msgs, KeyValue)>
        ("Console", "teleop_psm_selected",
         _ros_namespace + "teleop/teleop_psm_selected");
    events_bridge().AddPublisherFromEventWrite<prmKeyValue, CISST_RAL_MSG(diagnostic_msgs, KeyValue)>
        ("Console", "teleop_psm_unselected",
         _ros_namespace + "teleop/teleop_psm_unselected");

    events_bridge().AddSubscriberToCommandWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        ("Console", "set_volume",
         _ros_namespace + "set_volume");
    events_bridge().AddPublisherFromEventWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        ("Console", "volume",
         _ros_namespace + "volume");
    events_bridge().AddSubscriberToCommandWrite<vctDoubleVec, CISST_RAL_MSG(std_msgs, Float64MultiArray)>
        ("Console", "beep",
         _ros_namespace + "beep");
    events_bridge().AddSubscriberToCommandWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        ("Console", "string_to_speech",
         _ros_namespace + "string_to_speech");

    events_bridge().AddPublisherFromEventWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
        ("ConsoleOperatorPresent", "Button",
         _ros_namespace + "operator_present");
    m_connections.Add(events_bridge().GetName(), "ConsoleOperatorPresent",
                      m_console->GetName(), "OperatorPresent");
    events_bridge().AddPublisherFromEventWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
        ("ConsoleClutch", "Button",
         _ros_namespace + "clutch");
    m_connections.Add(events_bridge().GetName(), "ConsoleClutch",
                      m_console->GetName(), "Clutch");
    events_bridge().AddPublisherFromEventWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
        ("ConsoleCamera", "Button",
         _ros_namespace + "camera");
    m_connections.Add(events_bridge().GetName(), "ConsoleCamera",
                      m_console->GetName(), "Camera");

    subscribers_bridge().AddSubscriberToCommandWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
        ("Console", "emulate_operator_present",
         _ros_namespace + "emulate_operator_present");
    subscribers_bridge().AddSubscriberToCommandWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
        ("Console", "emulate_clutch",
         _ros_namespace + "emulate_clutch");
    subscribers_bridge().AddSubscriberToCommandWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
        ("Console", "emulate_camera",
         _ros_namespace + "emulate_camera");

    m_connections.Add(subscribers_bridge().GetName(), "Console",
                      m_console->GetName(), "Main");
    m_connections.Add(events_bridge().GetName(), "Console",
                      m_console->GetName(), "Main");
}

void dvrk::console::add_topics_endoscope_focus(void)
{
    const std::string _ros_namespace = "endoscope_focus/";
    const std::string _focus_component_name = m_console->mDaVinciEndoscopeFocus->GetName();

    // events
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_focus_component_name, "locked",
         _ros_namespace + "locked");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_focus_component_name, "focusing_in",
         _ros_namespace + "focusing_in");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_focus_component_name, "focusing_out",
         _ros_namespace + "focusing_out");

    // commands
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_focus_component_name, "lock",
         _ros_namespace + "lock");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_focus_component_name, "focus_in",
         _ros_namespace + "focus_in");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_focus_component_name, "focus_out",
         _ros_namespace + "focus_out");

    m_connections.Add(subscribers_bridge().GetName(), _focus_component_name,
                      _focus_component_name, "Control");
    m_connections.Add(events_bridge().GetName(), _focus_component_name,
                      _focus_component_name, "Control");
}

void dvrk::console::add_topics_io(void)
{
    const std::string _ros_namespace = "stats/io/";
    m_pub_bridge->AddPublisherFromCommandRead<mtsIntervalStatistics, CISST_RAL_MSG(cisst_msgs, IntervalStatistics)>
        ("io", "period_statistics",
         _ros_namespace + "period_statistics");
    m_pub_bridge->AddPublisherFromCommandRead<mtsIntervalStatistics, CISST_RAL_MSG(cisst_msgs, IntervalStatistics)>
        ("io", "period_statistics_read",
         _ros_namespace + "period_statistics_read");
    m_pub_bridge->AddPublisherFromCommandRead<mtsIntervalStatistics, CISST_RAL_MSG(cisst_msgs, IntervalStatistics)>
        ("io", "period_statistics_write",
         _ros_namespace + "period_statistics_write");

    m_connections.Add(m_pub_bridge->GetName(), "io",
                      m_console->m_IO_component_name, "Configuration");
}

void dvrk::console::add_topics_io(const double _publish_period_in_seconds,
                                  const bool _read_write)
{
    if (!m_io_bridge) {
        mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
        // io bridge uses an object factory based on list of interfaces provided by the IO component
        m_io_bridge = new mts_ros_crtk_robot_io_bridge("io-bridge", m_node_handle_ptr,
                                                       "io/", _publish_period_in_seconds,
                                                       /*tf*/ 0.0, _read_write, /* spin */ false);
        component_manager->AddComponent(m_io_bridge);
        component_manager->Connect("io-bridge", "RobotConfiguration",
                                   "io", "Configuration");
        m_io_bridge->Configure();
    }
}

void dvrk::console::add_topics_pid(const double _publish_period_in_seconds,
                                   const bool _read_write)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    for (auto & armPair : m_console->mArms) {
        const auto name = armPair.first;
        const auto arm = armPair.second;
        if (arm->expects_PID()) {
            // we need to create on PID bridge per PID component
            const std::string pid_component_name = name + "-PID";
            const std::string pid_bridge_name = "pid-bridge-" + name;
            mts_ros_crtk_controllers_pid_bridge * bridge
                = new mts_ros_crtk_controllers_pid_bridge(pid_bridge_name, m_node_handle_ptr,
                                                          5.0 * cmn_ms, /* spin */ false);
            bridge->bridge_interface_provided(pid_component_name, "Controller", "pid/" + name,
                                              _publish_period_in_seconds, /* tf */ 0.0,
                                              /* rw */ _read_write);
            component_manager->AddComponent(bridge);
            bridge->Connect();
        }
    }
}


void dvrk::console::add_topics_ecm_io(const std::string & _arm_name,
                                      const std::string & _io_component_name)
{
    // known events and corresponding ros topic
    const auto events = std::list<std::pair<std::string, std::string> >({
            {"ManipClutch", "manip_clutch"},
                {"SUJClutch", "suj_clutch"}});
    for (auto event : events) {
        std::string _interface_name = _arm_name + "-" + event.first;
        events_bridge().AddPublisherFromEventWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
            (_interface_name, "Button", _arm_name + "/io/" + event.second);
        m_connections.Add(events_bridge().GetName(), _interface_name,
                          _io_component_name, _interface_name);
    }
}

void dvrk::console::add_topics_psm_io(const std::string & _arm_name,
                                      const std::string & _io_component_name)
{
    // known events and corresponding ros topic
    const auto events = std::list<std::pair<std::string, std::string> >({
            {"ManipClutch", "manip_clutch"},
                {"SUJClutch", "suj_clutch"},
                    {"Adapter", "adapter"},
                        {"Tool", "tool"}});
    for (auto event : events) {
        std::string _interface_name = _arm_name + "-" + event.first;
        events_bridge().AddPublisherFromEventWrite<prmEventButton, CISST_RAL_MSG(sensor_msgs, Joy)>
            (_interface_name, "Button", _arm_name + "/io/" + event.second);
        m_connections.Add(events_bridge().GetName(), _interface_name,
                          _io_component_name, _interface_name);
    }
}

void dvrk::console::add_topics_suj_voltages(void)
{
    mtsManagerLocal * _component_manager = mtsManagerLocal::GetInstance();
    mtsComponent * _suj = _component_manager->GetComponent("SUJ");
    if (!_suj) {
        CMN_LOG_CLASS_INIT_WARNING << "add_topics_suj_voltages: no SUJ on this console!  option -s ignored!" << std::endl;
        return;
    }
    mtsROSBridge * _pub_bridge = new mtsROSBridge("SUJ-Voltages", 0.005 * cmn_s,
                                                node_handle_ptr());
    const auto arms = std::list<std::string>({"ECM", "PSM1", "PSM2", "PSM3"});
    for (auto arm : arms) {
        _pub_bridge->AddPublisherFromCommandRead<vctDoubleVec, CISST_RAL_MSG(sensor_msgs, JointState)>
            ("SUJ-" + arm, "GetVoltagesPrimary",
             "SUJ/" + arm + "/primary_voltage/measured_js");
        _pub_bridge->AddPublisherFromCommandRead<vctDoubleVec, CISST_RAL_MSG(sensor_msgs, JointState)>
            ("SUJ-" + arm, "GetVoltagesSecondary",
             "SUJ/" + arm + "/secondary_voltage/measured_js");
        m_connections.Add(_pub_bridge->GetName(), "SUJ-" + arm,
                          "SUJ", arm);
    }
    _component_manager->AddComponent(_pub_bridge);
}

void dvrk::console::add_topics_teleop_ecm(const std::string & _name)
{
    std::string _ros_namespace = _name;
    cisst_ral::clean_namespace(_ros_namespace);
    _ros_namespace += "/";

    // messages
    events_bridge().AddLogFromEventWrite(_name + "-log", "error",
                                         mtsROSEventWriteLog::ROS_LOG_ERROR);
    events_bridge().AddLogFromEventWrite(_name + "-log", "warning",
                                         mtsROSEventWriteLog::ROS_LOG_WARN);
    events_bridge().AddLogFromEventWrite(_name + "-log", "status",
                                         mtsROSEventWriteLog::ROS_LOG_INFO);
    // connect
    m_connections.Add(events_bridge().GetName(), _name + "-log",
                      _name, "Setting");

    // events
    events_bridge().AddPublisherFromEventWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_name, "desired_state", _ros_namespace + "desired_state");
    events_bridge().AddPublisherFromEventWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_name, "current_state", _ros_namespace + "current_state");
    events_bridge().AddPublisherFromEventWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_name, "scale", _ros_namespace + "scale");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_name, "following", _ros_namespace + "following");
    // connect
    m_connections.Add(events_bridge().GetName(), _name,
                      _name, "Setting");

    // commands
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, CISST_RAL_MSG(crtk_msgs, StringStamped)>
        (_name, "state_command",
         _ros_namespace + "state_command");
    subscribers_bridge().AddSubscriberToCommandWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_name, "set_scale",
         _ros_namespace + "set_scale");
    // connect
    m_connections.Add(subscribers_bridge().GetName(), _name,
                      _name, "Setting");
}

void dvrk::console::add_topics_teleop_psm(const std::string & _name)
{
    std::string _ros_namespace = _name;
    cisst_ral::clean_namespace(_ros_namespace);
    _ros_namespace += "/";

    // messages
    events_bridge().AddLogFromEventWrite(_name + "-log", "error",
                                         mtsROSEventWriteLog::ROS_LOG_ERROR);
    events_bridge().AddLogFromEventWrite(_name + "-log", "warning",
                                         mtsROSEventWriteLog::ROS_LOG_WARN);
    events_bridge().AddLogFromEventWrite(_name + "-log", "status",
                                         mtsROSEventWriteLog::ROS_LOG_INFO);
    // connect
    m_connections.Add(events_bridge().GetName(), _name + "-log",
                      _name, "Setting");

    // publisher
    m_pub_bridge->AddPublisherFromCommandRead<vctMatRot3, CISST_RAL_MSG(geometry_msgs, QuaternionStamped)>
        (_name, "alignment_offset",
         _ros_namespace + "alignment_offset");
    // connect
    m_connections.Add(m_pub_bridge->GetName(), _name,
                      _name, "Setting");

    // events
    events_bridge().AddPublisherFromEventWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_name, "desired_state", _ros_namespace + "desired_state");
    events_bridge().AddPublisherFromEventWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        (_name, "current_state", _ros_namespace + "current_state");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(sensor_msgs, Joy)>
        (_name, "rotation_locked",
         _ros_namespace + "rotation_locked");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(sensor_msgs, Joy)>
        (_name, "translation_locked",
         _ros_namespace + "translation_locked");
    events_bridge().AddPublisherFromEventWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_name, "scale", _ros_namespace + "scale");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_name, "following", _ros_namespace + "following");
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_name, "align_mtm", _ros_namespace + "align_mtm");
    // connect
    m_connections.Add(events_bridge().GetName(), _name,
                      _name, "Setting");

    // commands
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, CISST_RAL_MSG(crtk_msgs, StringStamped)>
        (_name, "state_command",
         _ros_namespace + "state_command");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_name, "lock_translation",
         _ros_namespace + "lock_translation");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_name, "lock_rotation",
         _ros_namespace + "lock_rotation");
    subscribers_bridge().AddSubscriberToCommandWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_name, "set_scale",
         _ros_namespace + "set_scale");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        (_name, "set_align_mtm",
         _ros_namespace + "set_align_mtm");
    // connect
    m_connections.Add(subscribers_bridge().GetName(), _name,
                      _name, "Setting");
}

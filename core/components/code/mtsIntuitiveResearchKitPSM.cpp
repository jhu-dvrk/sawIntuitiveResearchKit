/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>
#include <time.h>

// cisst
#include <sawIntuitiveResearchKit/robManipulatorPSMSnake.h>

#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/prmActuatorJointCouplingCheck.h>

class GravityCompensationPSM : public robGravityCompensation {
public:
    bool configure(std::string physical_dh_file);
    std::string error(void);

    vctVec compute(const prmStateJoint& state, vct3 gravity) override;
private:
    robManipulator physical_model;
    std::string error_message;
};

bool GravityCompensationPSM::configure(std::string physical_dh_file)
{
    robManipulator::Errno err = physical_model.LoadRobot(physical_dh_file);
    if (err != robManipulator::ESUCCESS) {
        error_message = physical_model.mLastError;
        return false;
    }

    // make sure we have expected number of links
    if (physical_model.links.size() == 7) {
        return true;
    } else {
        error_message = "PSM GC kinematics does not match expected number of links for Si";
        return false;
    }
}

std::string GravityCompensationPSM::error(void)
{
    return error_message;
}

vctVec GravityCompensationPSM::compute(const prmStateJoint& state, vct3 gravity)
{
    vctDoubleVec qd(7, 0.0);
    auto j = state.Position();
    // convert virtual joint positions to physical
    vctDoubleVec q(7, j[0], 0.0, j[1], -j[1], j[1], 0.5*j[2], 0.5*j[2]);
    vctDoubleVec predicted_efforts = physical_model.CCG_MDH(q, qd, gravity);
    vctDoubleVec efforts(state.Position().size(), 0.0);
    efforts[0] = predicted_efforts[0];
    efforts[1] = predicted_efforts[2] - predicted_efforts[3] + predicted_efforts[4];
    efforts[2] = predicted_efforts[6];

    return efforts;
}

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitPSM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds):
    mtsIntuitiveResearchKitArm(componentName, periodInSeconds),
    m_tool_list(*this)
{
    Init();
}

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsIntuitiveResearchKitArm(arg),
    m_tool_list(*this)
{
    Init();
}

// need to define destructor after definition of GravityCompensationPSM is available
mtsIntuitiveResearchKitPSM::~mtsIntuitiveResearchKitPSM() = default;

void mtsIntuitiveResearchKitPSM::set_simulated(void)
{
    mtsIntuitiveResearchKitArm::set_simulated();
    // in simulation mode, we don't need clutch, adapter, tool IO nor Dallas
    RemoveInterfaceRequired("arm_clutch");
    RemoveInterfaceRequired("adapter");
    RemoveInterfaceRequired("tool");
    RemoveInterfaceRequired("dallas");
    // for Si systems, remove a few more interfaces
    if (m_generation == dvrk::generation::Si) {
        RemoveInterfaceRequired("SUJ_clutch");
        RemoveInterfaceRequired("SUJ_clutch_2");
        RemoveInterfaceRequired("SUJ_brake");
    }
}

void mtsIntuitiveResearchKitPSM::set_generation(const dvrk::generation generation)
{
    mtsIntuitiveResearchKitArm::set_generation(generation);
    // for S/si, add SUJClutch interface
    if ((generation == dvrk::generation::Si)
        && !m_simulated) {
        auto interfaceRequired = AddInterfaceRequired("SUJ_clutch");
        if (interfaceRequired) {
            interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch, this, "Button");
        }
        interfaceRequired = AddInterfaceRequired("SUJ_clutch_2");
        if (interfaceRequired) {
            interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch, this, "Button");
        }
        interfaceRequired = AddInterfaceRequired("SUJ_brake");
        if (interfaceRequired) {
            interfaceRequired->AddFunction("SetValue", SUJClutch.Brake);
        }
    } else {
        if (GetInterfaceProvided("SUJ_clutch")) {
            RemoveInterfaceRequired("SUJ_clutch");
        }
    }
}

void mtsIntuitiveResearchKitPSM::load_tool_list(const cmnPath & path,
                                                const std::string & indexFile)
{
    m_tool_list.Load(path, indexFile);
}

void mtsIntuitiveResearchKitPSM::tool_list_size(size_t & size) const
{
    size = m_tool_list.size();
}

void mtsIntuitiveResearchKitPSM::tool_name(const size_t & index, std::string & name) const
{
    name = m_tool_list.Name(index);
}

void mtsIntuitiveResearchKitPSM::tool_full_description(const size_t & index, std::string & description) const
{
    description = m_tool_list.FullDescription(index);
}

void mtsIntuitiveResearchKitPSM::PostConfigure(const Json::Value & jsonConfig,
                                               const cmnPath & configPath,
                                               const std::string & filename)
{
    // joint configuration when instrument is not present. First make sure
    // the DH are loaded for the first 3 joints and use the values
    // from the manipulator. For last 4, values are hard coded in
    // this method

    // load default tool index
    load_tool_list(configPath);

    // extra tool definitions
    const auto jsonToolIndexFile = jsonConfig["custom_tool_index"];
    if (!jsonToolIndexFile.isNull()) {
        const auto toolIndexFile = jsonToolIndexFile.asString();
        auto fullname = configPath.Find(toolIndexFile);
        if (fullname == "") {
            CMN_LOG_CLASS_INIT_ERROR << "PostConfigure: " << this->GetName()
                                     << " using file \"" << filename << "\" can't find tool index file \""
                                     << toolIndexFile << "\" in path: "
                                     << configPath << std::endl;
            exit(EXIT_FAILURE);
        }
        load_tool_list(configPath, toolIndexFile);
    }

    // tool detection
    const auto jsonToolDetection = jsonConfig["tool_detection"];
    if (!jsonToolDetection.isNull()) {
        std::string toolDetection = jsonToolDetection.asString();
        m_tool_detection = mtsIntuitiveResearchKitToolTypes::DetectionFromString(toolDetection);
        if (m_tool_detection == mtsIntuitiveResearchKitToolTypes::FIXED) {
            const auto jsonFixedTool = jsonConfig["tool"];
            if (!jsonFixedTool.isNull()) {
                const std::string fixedTool = jsonFixedTool.asString();
                // check if the tool is in the supported list (string name)
                const bool found = m_tool_list.Find(fixedTool, m_tool_index);
                if (!found) {
                    CMN_LOG_CLASS_INIT_ERROR << "PostConfigure: " << this->GetName()
                                             << ", \"" << fixedTool << "\" found in file \""
                                             << filename << "\" is not a supported type." << std::endl
                                             << "Supported tool types are:\n" << m_tool_list.PossibleNames("\n") << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "PostConfigure: " << this->GetName()
                                         << " can't find field \"tool\" in file \""
                                         << filename << "\" which is required since \"tool-detection\" is set to \"FIXED\"" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    } else {
        m_tool_detection = mtsIntuitiveResearchKitToolTypes::AUTOMATIC;
    }

    // ask to set pitch if not already defined
    if ((generation() == dvrk::generation::Si)
        && (m_mounting_pitch > std::numeric_limits<double>::max())
        ) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName() << std::endl
                                 << "\"mounting_pitch\" was not defined in \"" << filename << "\"" << std::endl
                                 << "If your PSM is mounted on the SUJ you should add it using: " << std::endl
                                 << " \"mounting_pitch\": " << std::to_string(-45.0)
                                 << " // " << std::to_string(-45.0 * cmn180_PI) << " degrees for PSM1 and PSM2"
                                 << "Or:"
                                 << " \"mounting_pitch\": " << std::to_string(-15.0)
                                 << " // " << std::to_string(-15.0 * cmn180_PI) << " degrees for PSM3"
                                 << std::endl;
        exit(EXIT_FAILURE);
    }
}

void mtsIntuitiveResearchKitPSM::ConfigureGC(const Json::Value & jsonConfig,
                                               const cmnPath & configPath,
                                               const std::string & filename)
{
    std::string physical_dh_name;
    const auto jsonPhysicalDH = jsonConfig["kinematic_gc"];
    if (!jsonPhysicalDH.isNull()) {
        physical_dh_name = jsonPhysicalDH.asString();
    } else if (m_generation == dvrk::generation::Si) {
        CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureGC: " << GetName() << ": no GC kinematics specified, using default for PSM Si" << std::endl;
        physical_dh_name = "kinematic/PSM_Si_physical.json";
    } else {
        CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureGC: " << GetName() << ": no GC kinematics specified, so gravity compensation is not available" << std::endl;
        return;
    }

    const auto physical_dh = configPath.Find(physical_dh_name);
    if (physical_dh == "") {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureGC: " << this->GetName()
                                 << " using file \"" << filename << "\", can't find GC kinematics file \""
                                 << physical_dh << "\"" << std::endl;
        exit(EXIT_FAILURE);
    }

    m_gc = std::make_unique<GravityCompensationPSM>();
    bool ok = m_gc->configure(physical_dh);
    if (ok) {
        gravity_compensation = m_gc.get();
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureGC: " << this->GetName()
                                 << " using GC kinematics file \"" << physical_dh << "\", got error \""
                                 << m_gc->error() << "\"" << std::endl;
        exit(EXIT_FAILURE);
    }
}

bool mtsIntuitiveResearchKitPSM::ConfigureTool(const std::string & filename)
{
    std::string fullFilename;

    // try to locate the file based on tool type
    if (cmnPath::Exists(filename)) {
        fullFilename = filename;
    } else {
        // construct path using working directory and share/arm
        cmnPath path(cmnPath::GetWorkingDirectory());
        // find the file in tool
        path.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR) + "/tool", cmnPath::TAIL);
        // find file if specified as share/<system>/...
        path.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR), cmnPath::TAIL);
        // finally, default installation directory
        path.Add(mtsIntuitiveResearchKit::DefaultInstallationDirectory + "/tool", cmnPath::TAIL);

        fullFilename = path.Find(filename);
        // still not found, try to add suffix to search again
        if (fullFilename == "") {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": failed to locate tool file for \""
                                     << filename << "\"" << std::endl;
            return false;
        }
    }

    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(fullFilename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": failed to parse configuration file \""
                                     << fullFilename << "\"\n"
                                     << jsonReader.getFormattedErrorMessages();
            return false;
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                                   << " using file \"" << fullFilename << "\"" << std::endl
                                   << "----> content of configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

        m_snake_like = false;
        const Json::Value snakeLike = jsonConfig["snake-like"];
        if (!snakeLike.isNull()) {
            m_snake_like = snakeLike.asBool();
        }

        // snake require the derived manipulator class so we might
        // have to delete create manipulator

        // preserve Rtw0 just in case we need to create a new instance
        // of robManipulator
        CMN_ASSERT(Manipulator);
        vctFrm4x4 oldRtw0 = Manipulator->Rtw0;
        bool newInstance = false;

        if (m_snake_like) {
            // maybe we already have it?
            if (!dynamic_cast<robManipulatorPSMSnake *>(this->Manipulator)) {
                delete this->Manipulator;
                this->Manipulator = new robManipulatorPSMSnake();
                newInstance = true;
            }
        } else {
            // make sure we have the base robManipulator class
            if (dynamic_cast<robManipulatorPSMSnake *>(this->Manipulator)) {
                delete this->Manipulator;
                this->Manipulator = new robManipulator();
                newInstance = true;
            }
        }

        // configure new instance and restore Rtw0 in case user have
        // overriden the content of config file
        if (newInstance) {
            ConfigureDH(mConfigurationFile);
            Manipulator->Rtw0.Assign(oldRtw0);
        }

        // remove tool tip offset
        Manipulator->DeleteTools();
        // in any case, we just need the first 3 links
        Manipulator->Truncate(3);

        // now configure the links specific to the tool
        ConfigureDH(jsonConfig, fullFilename, true /* ignore coupling, we load this later */);

        // check that the kinematic chain length makes sense
        size_t expectedNumberOfJoint;
        if (m_snake_like) {
            expectedNumberOfJoint = 8;
        } else {
            expectedNumberOfJoint = 6;
        }
        size_t numberOfJointsLoaded = this->Manipulator->links.size();

        if (expectedNumberOfJoint != numberOfJointsLoaded) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": incorrect number of joints (DH), found "
                                     << numberOfJointsLoaded << ", expected " << expectedNumberOfJoint
                                     << std::endl;
            return false;
        }

        // load tool tip transform if any (with warning)
        const Json::Value jsonToolTip = jsonConfig["tooltip_offset"];
        if (jsonToolTip.isNull()) {
            CMN_LOG_CLASS_INIT_WARNING << "ConfigureTool " << this->GetName()
                                       << ": can find \"tooltip_offset\" data in \"" << fullFilename << "\"" << std::endl;
        } else {
            cmnDataJSON<vctFrm4x4>::DeSerializeText(ToolOffsetTransformation, jsonToolTip);
            ToolOffset = new robManipulator(ToolOffsetTransformation);
            Manipulator->Attach(ToolOffset);
        }

        // keep info in log
        std::stringstream dhResult;
        this->Manipulator->PrintKinematics(dhResult);
        CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureTool " << this->GetName()
                                   << ": loaded kinematics" << std::endl << dhResult.str() << std::endl;
        // update ConfigurationJointKinematic from manipulator
        update_configuration_js();
        // resize data members using kinematics (jacobians and effort vectors)
        ResizeKinematicsData();

        // load coupling information (required)
        const Json::Value jsonCoupling = jsonConfig["coupling"];
        if (jsonCoupling.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"coupling\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        }

        // read 4x4 coupling for last 3 DOFs and jaws
        prmActuatorJointCoupling toolCoupling, armCoupling;
        cmnDataJSON<prmActuatorJointCoupling>::DeSerializeText(toolCoupling,
                                                               jsonCoupling);
        // build a coupling matrix for all 7 actuators/dofs
        armCoupling.ActuatorToJointPosition().ForceAssign(vctDynamicMatrix<double>::Eye(number_of_joints()));
        // assign 4x4 matrix starting at position 3, 3
        armCoupling.ActuatorToJointPosition().Ref(4, 4, 3, 3).Assign(toolCoupling.ActuatorToJointPosition());
        // update coupling matrix
        prmActuatorJointCouplingCheck(number_of_joints(),
                                      number_of_joints(),
                                      armCoupling, m_coupling);
        m_has_coupling = true;

        // load jaw data, i.e. joint and torque limits
        const Json::Value jsonJaw = jsonConfig["jaw"];
        if (jsonJaw.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        }
        const Json::Value jsonJawQMin = jsonJaw["qmin"];
        if (jsonJawQMin.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::qmin\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        } else {
            m_jaw_configuration_js.PositionMin().SetSize(1);
            m_jaw_configuration_js.PositionMin().at(0) = jsonJawQMin.asDouble();
        }
        const Json::Value jsonJawQMax = jsonJaw["qmax"];
        if (jsonJawQMax.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::qmax\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        } else {
            m_jaw_configuration_js.PositionMax().SetSize(1);
            m_jaw_configuration_js.PositionMax().at(0) = jsonJawQMax.asDouble();
        }
        const Json::Value jsonJawFTMax = jsonJaw["ftmax"];
        if (jsonJawFTMax.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::ftmax\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        } else {
            m_jaw_configuration_js.EffortMin().SetSize(1);
            m_jaw_configuration_js.EffortMax().SetSize(1);
            m_jaw_configuration_js.EffortMax().at(0) = jsonJawFTMax.asDouble();
            m_jaw_configuration_js.EffortMin().at(0) = -jsonJawFTMax.asDouble();
        }

        m_jaw_configuration_js.Name().resize(1);
        m_jaw_configuration_js.Name().at(0) = "jaw";
        m_jaw_configuration_js.Type().SetSize(1);
        m_jaw_configuration_js.Type().at(0) = CMN_JOINT_REVOLUTE;

        // load lower/upper position used to engage the tool(required)
        const Json::Value jsonEngagePosition = jsonConfig["tool_engage_position"];
        if (jsonEngagePosition.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"tool_engage_position\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        }
        // lower
        cmnDataJSON<vctDoubleVec>::DeSerializeText(m_tool_engage_lower_position,
                                                   jsonEngagePosition["lower"]);
        if (m_tool_engage_lower_position.size() != 4) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": \"tool_engage_position\" : \"lower\" must contain 4 elements in \""
                                     << fullFilename << "\"" << std::endl;
            return false;
        }
        // upper
        cmnDataJSON<vctDoubleVec>::DeSerializeText(m_tool_engage_upper_position,
                                                   jsonEngagePosition["upper"]);
        if (m_tool_engage_upper_position.size() != 4) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": \"tool_engage_position\" : \"upper\" must contain 4 elements in \""
                                     << fullFilename << "\"" << std::endl;
            return false;
        }

    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName() << ": make sure the file \""
                                 << fullFilename << "\" is in JSON format" << std::endl;
        return false;
    }

    return true;
}


void mtsIntuitiveResearchKitPSM::UpdateStateJointKinematics(void)
{
    // if there is no tool, report joints as PID joints
    if (!is_cartesian_ready()) {
        mtsIntuitiveResearchKitArm::UpdateStateJointKinematics();
        return;
    }

    const size_t nbPIDJoints = m_pid_measured_js.Name().size();
    const size_t jawIndex = nbPIDJoints - 1;

    m_jaw_measured_js.Position().at(0) = m_pid_measured_js.Position().at(jawIndex);
    m_jaw_measured_js.Velocity().at(0) = m_pid_measured_js.Velocity().at(jawIndex);
    m_jaw_measured_js.Effort().at(0)   = m_pid_measured_js.Effort().at(jawIndex);
    m_jaw_measured_js.Timestamp() = m_pid_measured_js.Timestamp();
    m_jaw_measured_js.Valid() = m_pid_measured_js.Valid();

    m_jaw_setpoint_js.Position().at(0) = m_pid_setpoint_js.Position().at(jawIndex);
    m_jaw_setpoint_js.Effort().at(0)   = m_pid_setpoint_js.Effort().at(jawIndex);
    m_jaw_setpoint_js.Timestamp() = m_pid_setpoint_js.Timestamp();
    m_jaw_setpoint_js.Valid() = m_pid_setpoint_js.Timestamp();

    if (!m_snake_like) {

        // most tool, copy first n joints (6) from PID for kinematics
        // measured p/v/e
        m_kin_measured_js.Position().Assign(m_pid_measured_js.Position().Ref(number_of_joints_kinematics()));
        m_kin_measured_js.Velocity().Assign(m_pid_measured_js.Velocity().Ref(number_of_joints_kinematics()));
        m_kin_measured_js.Effort().Assign(m_pid_measured_js.Effort().Ref(number_of_joints_kinematics()));
        m_kin_measured_js.Timestamp() = m_pid_measured_js.Timestamp();
        m_kin_measured_js.Valid() = m_pid_measured_js.Valid();

        // setpoint p/e
        m_kin_setpoint_js.Position().Assign(m_pid_setpoint_js.Position().Ref(number_of_joints_kinematics()));
        m_kin_setpoint_js.Velocity().Assign(m_pid_setpoint_js.Velocity().Ref(number_of_joints_kinematics()));
        m_kin_setpoint_js.Effort().Assign(m_pid_setpoint_js.Effort().Ref(number_of_joints_kinematics()));
        m_kin_setpoint_js.Timestamp() = m_pid_setpoint_js.Timestamp();
        m_kin_setpoint_js.Valid() = m_pid_setpoint_js.Valid();

    } else {

        // measured p/v/e
        m_kin_measured_js.Position().Assign(m_pid_measured_js.Position(), 4);
        m_kin_measured_js.Position().at(4) = m_kin_measured_js.Position().at(7) = m_pid_measured_js.Position().at(4) / 2.0;
        m_kin_measured_js.Position().at(5) = m_kin_measured_js.Position().at(6) = m_pid_measured_js.Position().at(5) / 2.0;

        m_kin_measured_js.Velocity().Assign(m_pid_measured_js.Velocity(), 4);
        m_kin_measured_js.Velocity().at(4) = m_kin_measured_js.Velocity().at(7) = m_pid_measured_js.Velocity().at(4) / 2.0;
        m_kin_measured_js.Velocity().at(5) = m_kin_measured_js.Velocity().at(6) = m_pid_measured_js.Velocity().at(5) / 2.0;

        m_kin_measured_js.Effort().Assign(m_pid_measured_js.Effort(), 4);
        m_kin_measured_js.Effort().at(4) = m_kin_measured_js.Effort().at(7) = m_pid_measured_js.Effort().at(4) / 2.0;
        m_kin_measured_js.Effort().at(5) = m_kin_measured_js.Effort().at(6) = m_pid_measured_js.Effort().at(5) / 2.0;
        m_kin_measured_js.Timestamp() = m_pid_measured_js.Timestamp();
        m_kin_measured_js.Valid() = m_pid_measured_js.Valid();

        // setpoint p/e
        m_kin_setpoint_js.Position().Assign(m_pid_setpoint_js.Position(), 4);
        m_kin_setpoint_js.Position().at(4) = m_kin_setpoint_js.Position().at(7) = m_pid_setpoint_js.Position().at(4) / 2.0;
        m_kin_setpoint_js.Position().at(5) = m_kin_setpoint_js.Position().at(6) = m_pid_setpoint_js.Position().at(5) / 2.0;
        std::cerr << CMN_LOG_DETAILS << " ------- need to add code to generate setpoint_js.Velocity " << std::endl;
        m_kin_setpoint_js.Effort().Assign(m_pid_measured_js.Effort(), 4);
        m_kin_setpoint_js.Effort().at(4) = m_kin_setpoint_js.Effort().at(7) = m_pid_setpoint_js.Effort().at(4) / 2.0;
        m_kin_setpoint_js.Effort().at(5) = m_kin_setpoint_js.Effort().at(6) = m_pid_setpoint_js.Effort().at(5) / 2.0;
        m_kin_setpoint_js.Timestamp() = m_pid_setpoint_js.Timestamp();
        m_kin_setpoint_js.Valid() = m_pid_setpoint_js.Valid();
    }
}

void mtsIntuitiveResearchKitPSM::ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID)
{
    if (is_cartesian_ready()) {
        // tool is present
        if (m_snake_like) {
            CMN_ASSERT(jointsKinematics.size() == 8);
            jointsPID.Assign(jointsKinematics, 4);
            // Test if position 4 and 7 are very much apart; throw error maybe ?
            jointsPID.at(4) = jointsKinematics.at(4) + jointsKinematics.at(7);
            // Same goes for 5 and 6
            jointsPID.at(5) = jointsKinematics.at(5) + jointsKinematics.at(6);
        } else {
            CMN_ASSERT(jointsKinematics.size() >= 6);
            jointsPID.Assign(jointsKinematics, 6);
        }
    } else {
        // joint space, no tool yet so we can control all 7 actuators
        CMN_ASSERT(jointsKinematics.size() == 7);
        jointsPID.Assign(jointsKinematics);
    }
}

robManipulator::Errno mtsIntuitiveResearchKitPSM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal) const
{
    // make sure we are away from RCM point, create a new goal on sphere around RCM point (i.e. origin)
    double distanceToRCM = cartesianGoal.Translation().Norm();
    double currentDepth = jointSet.at(2);

    // if too close to zero we're going to run into issue in any case
    if (distanceToRCM < 1.0 * cmn_mm) {
        m_arm_interface->SendWarning(GetName() + ": InverseKinematics, can't solve IK too close to RCM");
        return robManipulator::EFAILURE;
    }

    // IK
    robManipulator::Errno Err = Manipulator->InverseKinematics(jointSet, cartesianGoal);;

    // check equality constraint for snake like kinematic
    if (m_snake_like) {
        // Check for equality Snake joints (4,7) and (5,6)
        if (fabs(jointSet.at(4) - jointSet.at(7)) > 0.00001 ||
            fabs(jointSet.at(5) - jointSet.at(6)) > 0.00001) {
            m_arm_interface->SendWarning(GetName() + ": InverseKinematics, equality constraint violated");
        }
    }

    // Find closest solution mod 2 Pi for roll along shaft
    if (Err == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = m_kin_measured_js.Position().at(3) - jointSet.at(3);
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet.at(3) = jointSet.at(3) + differenceInTurns * 2.0 * cmnPI;

        // project away from RCM if not safe, using axis at end of shaft
        vctFrm4x4 f4;
        if (Manipulator->links.size() >= 4) {
            f4 = Manipulator->ForwardKinematics(jointSet, 4);
        } else {
            f4 = Manipulator->ForwardKinematics(jointSet);
        }
        distanceToRCM = f4.Translation().Norm();

        // if not far enough, distance for axis 4 is fully determine by insertion joint so add to it
        if (distanceToRCM < mtsIntuitiveResearchKit::PSM::SafeDistanceFromRCM) {
            // two cases based in current depth, were we past min depth or not - to do this we need to compute the minimum depth using j2.
            const double minDepth = jointSet.at(2) + (mtsIntuitiveResearchKit::PSM::SafeDistanceFromRCM - distanceToRCM);
            // if we are already too close to RCM, simply prevent to get closer
            if (currentDepth <= minDepth) {
                jointSet.at(2) = std::max(currentDepth, jointSet.at(2));
            } else {
                // else, make sure we don't go deeper
                jointSet.at(2) = minDepth;
            }
        }
        return robManipulator::ESUCCESS;
    }

    return robManipulator::EFAILURE;
}

bool mtsIntuitiveResearchKitPSM::is_safe_for_cartesian_control(void) const
{
    vctFrm4x4 f4;
    if (Manipulator->links.size() >= 4) {
        f4 = Manipulator->ForwardKinematics(m_kin_measured_js.Position(), 4);
    } else {
        f4 = Manipulator->ForwardKinematics(m_kin_measured_js.Position());
    }
    const double distanceToRCM = f4.Translation().Norm();
    return (distanceToRCM >=  (mtsIntuitiveResearchKit::PSM::SafeDistanceFromRCM
                               - mtsIntuitiveResearchKit::PSM::SafeDistanceFromRCMBuffer));
}

void mtsIntuitiveResearchKitPSM::Init(void)
{
    // main initialization from base type
    mtsIntuitiveResearchKitArm::Init();

    // state machine specific to PSM, see base class for other states
    mArmState.AddState("ENGAGING_ADAPTER");
    mArmState.AddState("ENGAGING_TOOL");
    mArmState.AddState("TOOL_ENGAGED");
    mArmState.AddState("MANUAL");

    // after arm homed
    mArmState.SetEnterCallback("HOMED",
                               &mtsIntuitiveResearchKitPSM::EnterHomed,
                               this);
    mArmState.SetTransitionCallback("HOMED",
                                    &mtsIntuitiveResearchKitPSM::TransitionHomed,
                                    this);
    mArmState.SetEnterCallback("ENGAGING_ADAPTER",
                               &mtsIntuitiveResearchKitPSM::EnterEngagingAdapter,
                               this);
    mArmState.SetRunCallback("ENGAGING_ADAPTER",
                             &mtsIntuitiveResearchKitPSM::RunEngagingAdapter,
                             this);
    mArmState.SetEnterCallback("ENGAGING_TOOL",
                               &mtsIntuitiveResearchKitPSM::EnterEngagingTool,
                               this);
    mArmState.SetRunCallback("ENGAGING_TOOL",
                             &mtsIntuitiveResearchKitPSM::RunEngagingTool,
                             this);
    mArmState.SetEnterCallback("TOOL_ENGAGED",
                               &mtsIntuitiveResearchKitPSM::EnterToolEngaged,
                               this);
    mArmState.SetTransitionCallback("TOOL_ENGAGED",
                                    &mtsIntuitiveResearchKitPSM::TransitionToolEngaged,
                                    this);
    mArmState.SetEnterCallback("MANUAL",
                               &mtsIntuitiveResearchKitPSM::EnterManual,
                               this);
    mArmState.SetRunCallback("MANUAL",
                             &mtsIntuitiveResearchKitPSM::RunManual,
                             this);
    mArmState.SetLeaveCallback("MANUAL",
                               &mtsIntuitiveResearchKitPSM::LeaveManual,
                               this);

    // initialize trajectory data, last 4 tweaked for engage procedures
    m_trajectory_j.v_max.Ref(2, 0).SetAll(90.0 * cmnPI_180); // degrees per second
    m_trajectory_j.v_max.Element(2) = 0.2; // m per second
    m_trajectory_j.v_max.Ref(4, 3).SetAll(3.0 * 360.0 * cmnPI_180);
    m_trajectory_j.a_max.Ref(2, 0).SetAll(90.0 * cmnPI_180);
    m_trajectory_j.a_max.Element(2) = 0.2; // m per second ^ 2
    m_trajectory_j.a_max.Ref(4, 3).SetAll(2.0 * 360.0 * cmnPI_180);
    m_trajectory_j.goal_tolerance.SetAll(3.0 * cmnPI_180); // hard coded to 3 degrees

    // default PID tracking errors
    PID.measured_setpoint_tolerance.SetSize(number_of_joints());
    // first two rotations
    PID.measured_setpoint_tolerance.Ref(2, 0).SetAll(20.0 * cmnPI_180); // 2 elements starting at 0 -> 0 1
    // translation
    PID.measured_setpoint_tolerance.Element(2) = 20.0 * cmn_mm; // 20 mm -> 2
    // shaft rotation and tool orientation
    PID.measured_setpoint_tolerance.Ref(3, 3).SetAll(35.0 * cmnPI_180); // 3 elements starting at 3 -> 3, 4, 5
    // jaws, allow more since we use PID large errors to apply large torques
    PID.measured_setpoint_tolerance.Element(6) = 90.0 * cmnPI_180;
    mtsInterfaceRequired * interfaceRequired;

    // Main interface should have been created by base class init
    CMN_ASSERT(m_arm_interface);
    m_jaw_measured_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_jaw_measured_js, "jaw/measured_js");

    m_jaw_setpoint_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_jaw_setpoint_js, "jaw/setpoint_js");

    m_jaw_servo_jf = 0.0;

    // state table for configuration
    mStateTableConfiguration.AddData(m_jaw_configuration_js, "jaw/configuration_js");

    // jaw interface
    m_arm_interface->AddCommandReadState(this->StateTable, m_jaw_measured_js, "jaw/measured_js");
    m_arm_interface->AddCommandReadState(this->StateTable, m_jaw_setpoint_js, "jaw/setpoint_js");
    m_arm_interface->AddCommandReadState(this->mStateTableConfiguration,
                                         m_jaw_configuration_js, "jaw/configuration_js");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_servo_jp, this, "jaw/servo_jp");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_move_jp, this, "jaw/move_jp");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_servo_jf, this, "jaw/servo_jf");

    // tool specific interface
    m_arm_interface->AddCommandRead(&mtsIntuitiveResearchKitPSM::tool_list_size, this, "tool_list_size");
    m_arm_interface->AddCommandQualifiedRead(&mtsIntuitiveResearchKitPSM::tool_name, this, "tool_name");
    m_arm_interface->AddCommandQualifiedRead(&mtsIntuitiveResearchKitPSM::tool_full_description, this, "tool_full_description");

    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::set_adapter_present, this, "emulate_adapter_present");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::emulate_tool_present, this, "emulate_tool_present");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::set_tool_type, this, "set_tool_type");
    m_arm_interface->AddEventWrite(ToolEvents.tool_type, "tool_type", std::string());
    m_arm_interface->AddEventVoid(ToolEvents.tool_type_request, "tool_type_request");

    m_arm_interface->AddEventWrite(ClutchEvents.ManipClutch, "ManipClutch", prmEventButton());

    // Event Adapter engage: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("adapter");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetButton", Adapter.GetButton);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerAdapter, this, "Button");
    }

    // Event Tool engage: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("tool");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetButton", Tool.GetButton);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerTool, this, "Button");
    }

    // ManipClutch: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("arm_clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerManipClutch, this, "Button");
    }

    // Dallas: read tool type from Dallas Chip
    interfaceRequired = AddInterfaceRequired("dallas");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("TriggerRead", Dallas.TriggerRead);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerToolType, this, "ToolType");
    }
}

bool mtsIntuitiveResearchKitPSM::is_homed(void) const
{
    if (Tool.IsPresent || Tool.IsEmulated) {
        return m_powered && m_encoders_biased_from_pots && Adapter.IsEngaged && Tool.IsEngaged;
    }
    if (Adapter.IsPresent) {
        return m_powered && m_encoders_biased_from_pots && Adapter.IsEngaged;
    }
    return m_powered && m_encoders_biased_from_pots;
}

void mtsIntuitiveResearchKitPSM::unhome(void)
{
    if (Tool.IsPresent || Tool.IsEmulated) {
        Tool.IsEngaged = false;
        return;
    }
    if (Adapter.IsPresent || Adapter.IsEmulated) {
        Adapter.IsEngaged = false;
        return;
    }
    // to force re-bias on pots
    m_re_home = true;
    m_encoders_biased_from_pots = false;
}

bool mtsIntuitiveResearchKitPSM::is_joint_ready(void) const
{
    return m_powered && m_encoders_biased_from_pots;
}

bool mtsIntuitiveResearchKitPSM::is_cartesian_ready(void) const
{
    return m_powered && m_encoders_biased_from_pots && Tool.IsEngaged;
}

void mtsIntuitiveResearchKitPSM::SetGoalHomingArm(void)
{
    // if simulated, start at zero but insert tool so it can be used in cartesian mode
    if (m_simulated) {
        m_trajectory_j.goal.SetAll(0.0);
        m_trajectory_j.goal.at(2) = 12.0 * cmn_cm;
        return;
    }

    // check if tool is present and if user wants to go to zero position
    Tool.GetButton(Tool.IsPresent);
    if (m_homing_goes_to_zero
        && !(Tool.IsPresent || Tool.IsEmulated)) {
        // move to zero position only there is no tool present
        m_trajectory_j.goal.SetAll(0.0);
    } else {
        // stay at current position by default
        m_trajectory_j.goal.Assign(m_pid_setpoint_js.Position());
    }
}


void mtsIntuitiveResearchKitPSM::EnterHomed(void)
{
    mtsIntuitiveResearchKitArm::EnterHomed();

    // event to propagate tool type based on configuration file
    if (m_simulated
        && (m_tool_detection == mtsIntuitiveResearchKitToolTypes::FIXED)) {
        prmEventButton fake_event;
        fake_event.SetType(prmEventButton::PRESSED);
        EventHandlerTool(fake_event);
    }
}


void mtsIntuitiveResearchKitPSM::TransitionHomed(void)
{
    if (!m_simulated) {
        Adapter.GetButton(Adapter.IsPresent);
        if (!(Adapter.IsPresent || Adapter.IsEmulated)) {
            Adapter.IsEngaged = false;
        }
    }
    if (!Adapter.IsEngaged &&
        (Adapter.IsPresent || Adapter.IsEmulated || m_simulated || Adapter.NeedEngage)) {
        mArmState.SetCurrentState("ENGAGING_ADAPTER");
    }

    if (!m_simulated) {
        Tool.GetButton(Tool.IsPresent);
        if (!(Tool.IsPresent || Tool.IsEmulated)) {
            Tool.IsEngaged = false;
        }
    }
    if (Adapter.IsEngaged && !Tool.IsEngaged) {
        if (!m_simulated) {
            if ((Tool.IsPresent || Tool.IsEmulated)) {
                // tool should be configured if automatic or fixed
                if (m_tool_configured) {
                    set_tool_present_and_configured(true, m_tool_configured);
                    mArmState.SetCurrentState("ENGAGING_TOOL");
                } else if ((m_tool_detection == mtsIntuitiveResearchKitToolTypes::MANUAL)
                           && !m_tool_type_requested) {
                    // manual is not sent until robot is ready
                    set_tool_present_and_configured(true, false);
                    m_tool_type_requested = true;
                    ToolEvents.tool_type_request();
                    m_arm_interface->SendWarning(this->GetName() + ": tool type requested from user");
                }
            }
        } else {
            // simulated case
            set_tool_present_and_configured(true, m_tool_configured);
            // check if tool is configured, i.e. fixed or manual
            if (m_tool_configured) {
                mArmState.SetCurrentState("ENGAGING_TOOL");
            } else {
                // request tool type if needed
                if (!m_tool_type_requested) {
                    m_tool_type_requested = true;
                    ToolEvents.tool_type_request();
                    m_arm_interface->SendWarning(this->GetName() + ": tool type requested from user");
                }
            }
        }
    }
}

void mtsIntuitiveResearchKitPSM::update_configuration_js_no_tool(void)
{
    // all vectors used to get data from Manipulator must match the
    // manipulator size, potentially 3, 6 or 8 (snake).  we can then
    // picj the first 3 elements to set the configuration_js.

    // names
    const size_t manipulator_size = Manipulator->links.size();
    m_configuration_js.Name().resize(number_of_joints());
    std::vector<std::string> name_tmp(manipulator_size);
    Manipulator->GetJointNames(name_tmp);
    // copy first 3 names to config vector
    for (size_t i = 0; i < 3; ++i) {
        m_configuration_js.Name().at(i) = name_tmp.at(i);
    }
    // overwrite last 4
    for (size_t i = 3; i < 7; ++i) {
        m_configuration_js.Name().at(i) = "disc_" + std::to_string(i - 2);
    }
    // for type, we can even ignore values loaded from config file
    m_configuration_js.Type().SetSize(number_of_joints());
    m_configuration_js.Type().SetAll(CMN_JOINT_REVOLUTE);
    m_configuration_js.Type().at(2) = CMN_JOINT_PRISMATIC;
    // position limits
    m_configuration_js.PositionMin().SetSize(number_of_joints());
    m_configuration_js.PositionMax().SetSize(number_of_joints());
    vctDoubleVec min_tmp(manipulator_size);
    vctDoubleVec max_tmp(manipulator_size);
    Manipulator->GetJointLimits(min_tmp, max_tmp);
    m_configuration_js.PositionMin().Ref(3, 0).Assign(min_tmp.Ref(3, 0));
    m_configuration_js.PositionMax().Ref(3, 0).Assign(max_tmp.Ref(3, 0));
    m_configuration_js.PositionMin().Ref(4, 3).SetAll(-mtsIntuitiveResearchKit::PSM::AdapterActuatorLimit);
    m_configuration_js.PositionMax().Ref(4, 3).SetAll( mtsIntuitiveResearchKit::PSM::AdapterActuatorLimit);
    // efforts
    m_configuration_js.EffortMin().SetSize(number_of_joints());
    m_configuration_js.EffortMax().SetSize(number_of_joints());
    Manipulator->GetFTMaximums(max_tmp);
    m_configuration_js.EffortMax().Ref(3, 0).Assign(max_tmp.Ref(3, 0));
    m_configuration_js.EffortMax().Ref(4, 3).SetAll(mtsIntuitiveResearchKit::PSM::DiskMaxTorque);
    m_configuration_js.EffortMin().Assign(-m_configuration_js.EffortMax());
}

void mtsIntuitiveResearchKitPSM::update_configuration_js(void)
{
    mStateTableConfiguration.Start();
    if (m_tool_present && m_tool_configured) {
        mtsIntuitiveResearchKitArm::update_configuration_js();
    } else {
        update_configuration_js_no_tool();
    }
    mStateTableConfiguration.Advance();
}

void mtsIntuitiveResearchKitPSM::EnterEngagingAdapter(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    // if simulated, nothing to do
    if (m_simulated) {
        return;
    }

    // after coupling is loaded, is it safe to engage?  If a tool is
    // present, the adapter is already engaged
    Tool.GetButton(Tool.IsPresent);
    if ((Tool.IsPresent || Tool.IsEmulated)) {
        // we can skip engage later
        Adapter.IsEngaged = true;
        mArmState.SetCurrentState("HOMED");
        return;
    }
    // if for some reason we don't need to engage, basically, adapter
    // was found before homing
    if (!Adapter.NeedEngage) {
        Adapter.IsEngaged = true;
        mArmState.SetCurrentState("HOMED");
        return;
    }

    // other case, initialize variables for adapter engage
    EngagingStage = 1;
    LastEngagingStage = 5;
    set_LED_pattern(mtsIntuitiveResearchKit::Blue200,
                    mtsIntuitiveResearchKit::Green200,
                    false, true);
}

void mtsIntuitiveResearchKitPSM::RunEngagingAdapter(void)
{
    if (m_simulated) {
        Adapter.NeedEngage = false;
        Adapter.IsEngaged = true;
        mArmState.SetCurrentState("HOMED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    if (EngagingStage == 1) {
        // configure PID to fail in case of tracking error
        PID.enforce_position_limits(false);
        vctDoubleVec tolerances(number_of_joints());
        // first two rotations and translation, in case someone is pushing/holding arm
        tolerances.Ref(2, 0).SetAll(10.0 * cmnPI_180); // 10 degrees
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        // tool/adapter gears should have little resistance?
        tolerances.Ref(4, 3).SetAll(45.0 * cmnPI_180);
        PID.set_measured_setpoint_tolerance(tolerances);
        servo_jp_internal(m_pid_setpoint_js.Position(), vctDoubleVec());
        // turn on PID
        PID.enable_joints(vctBoolVec(number_of_joints(), true));
        PID.enable_measured_setpoint_check(true);

        // make sure we start from current state
        m_servo_jp.Assign(m_pid_setpoint_js.Position());
        m_servo_jv.Assign(m_pid_setpoint_js.Velocity());

        // keep first two joint values as is
        m_trajectory_j.goal.Ref(2, 0).Assign(m_pid_setpoint_js.Position().Ref(2, 0));
        // sterile adapter should be raised up
        m_trajectory_j.goal[2] = 0.0;
        // set last 4 to -170.0
        m_trajectory_j.goal.Ref(4, 3).SetAll(-mtsIntuitiveResearchKit::PSM::AdapterEngageRange);
        m_trajectory_j.goal_v.SetAll(0.0);
        SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE);
        control_move_jp_on_start();
        EngagingStage = 2;
        return;
    }

    m_trajectory_j.Reflexxes.Evaluate(m_servo_jp,
                                      m_servo_jv,
                                      m_trajectory_j.goal,
                                      m_trajectory_j.goal_v);
    servo_jp_internal(m_servo_jp, m_servo_jv);

    const robReflexxes::ResultType trajectoryResult = m_trajectory_j.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (m_trajectory_j.end_time == 0.0) {
            m_trajectory_j.end_time = currentTime + m_trajectory_j.Reflexxes.Duration();
            m_homing_timer = m_trajectory_j.end_time;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        {
            // check if we were in last phase
            if (EngagingStage > LastEngagingStage) {
                Adapter.NeedEngage = false;
                Adapter.IsEngaged = true;
                set_LED_pattern(mtsIntuitiveResearchKit::Green200,
                                mtsIntuitiveResearchKit::Green200,
                                false, false);
                control_move_jp_on_stop(true); // goal reached
                mArmState.SetCurrentState("HOMED");
            } else {
                if (EngagingStage != LastEngagingStage) {
                    m_trajectory_j.goal.Ref(4, 3) *= -1.0; // toggle back and forth
                } else {
                    m_trajectory_j.goal.Ref(4, 3).SetAll(0.0); // back to zero position
                }
                m_trajectory_j.end_time = 0.0;
                std::stringstream message;
                message << this->GetName() << ": engaging adapter " << EngagingStage - 1 << " of " << LastEngagingStage - 1;
                m_arm_interface->SendStatus(message.str());
                EngagingStage++;
            }
        }
        break;

    default:
        m_arm_interface->SendError(this->GetName() + ": error while evaluating trajectory");
        SetDesiredState("FAULT");
        break;
    }
}

void mtsIntuitiveResearchKitPSM::EnterEngagingTool(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);

    // if for some reason we don't need to engage, basically, tool was
    // found before homing
    if (!m_simulated && !Tool.NeedEngage) {
        mArmState.SetCurrentState("TOOL_ENGAGED");
        return;
    }

    // other case, initialize variables for tool engage
    EngagingStage = 1;
    LastEngagingStage = 4;
    set_LED_pattern(mtsIntuitiveResearchKit::Blue200,
                    mtsIntuitiveResearchKit::Green200,
                    false, true);
}

void mtsIntuitiveResearchKitPSM::RunEngagingTool(void)
{
    if (m_simulated) {
        mArmState.SetCurrentState("TOOL_ENGAGED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    if (EngagingStage == 1) {
        // configure PID to fail in case of tracking error
        PID.enforce_position_limits(false);
        vctDoubleVec tolerances(number_of_joints());
        // first two rotations and translation, in case someone is pushing/holding arm
        tolerances.Ref(2, 0).SetAll(10.0 * cmnPI_180); // 10 degrees
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        // tool/adapter gears should have little resistance?
        tolerances.Ref(4, 3).SetAll(45.0 * cmnPI_180);
        PID.set_measured_setpoint_tolerance(tolerances);
        servo_jp_internal(m_pid_setpoint_js.Position(), vctDoubleVec());
        // turn on PID
        PID.enable_joints(vctBoolVec(number_of_joints(), true));
        PID.enable_measured_setpoint_check(true);

        // make sure we start from current state
        m_servo_jp.Assign(m_pid_setpoint_js.Position());
        m_servo_jv.Assign(m_pid_setpoint_js.Velocity());

        // check if the tool in outside the cannula, measured_cp is
        // not yet computed by get_robot_data so we need to compute
        // the FK ourselves.  This is fine for instruments with 6 dofs
        // but is not perfect for a snake-like instrument since we don't have 8 joint values
        vctFrm4x4 _measured_cp = Manipulator->ForwardKinematics(m_pid_measured_js.Position(), 6);
        const double distanceToRCM = _measured_cp.Translation().Norm();
        double depth;
        if (m_snake_like) {
            depth = mtsIntuitiveResearchKit::PSM::EngageDepthCannulaSnake;
        } else {
            depth = mtsIntuitiveResearchKit::PSM::EngageDepthCannula;
        }
        if (distanceToRCM >= depth) {
            std::string message = this->GetName();
            message.append(": tool tip is outside the cannula, assuming it doesn't need to \"engage\".");
            message.append("  If the tool is not engaged properly, move the sterile adapter all the way up and re-insert the tool.");
            CMN_LOG_CLASS_INIT_VERBOSE << "RunEngagingTool: distance to RCM is " << distanceToRCM
                                       << ", depth threshold is " << depth << std::endl;
            m_arm_interface->SendStatus(message);
            mArmState.SetCurrentState("TOOL_ENGAGED");
        }

        // keep first three joint values as is
        m_trajectory_j.goal.Ref(3, 0).Assign(m_pid_setpoint_js.Position().Ref(3, 0));
        // set last 4 to user preferences
        m_trajectory_j.goal.Ref(4, 3).Assign(m_tool_engage_lower_position);
        m_trajectory_j.goal_v.SetAll(0.0);
        SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE);
        control_move_jp_on_start();
        EngagingStage = 2;
        return;
    }

    m_trajectory_j.Reflexxes.Evaluate(m_servo_jp,
                                      m_servo_jv,
                                      m_trajectory_j.goal,
                                      m_trajectory_j.goal_v);
    servo_jp_internal(m_servo_jp, m_servo_jv);

    const robReflexxes::ResultType trajectoryResult = m_trajectory_j.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (m_trajectory_j.end_time == 0.0) {
            m_trajectory_j.end_time = currentTime + m_trajectory_j.Reflexxes.Duration();
            m_homing_timer = m_trajectory_j.end_time;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        {
            // check if we were in last phase
            if (EngagingStage > LastEngagingStage) {
                Tool.NeedEngage = false;
                control_move_jp_on_stop(true); // goal reached
                mArmState.SetCurrentState("TOOL_ENGAGED");
            } else {
                if (EngagingStage != LastEngagingStage) {
                    // toggle between lower and upper
                    if (EngagingStage % 2 == 0) {
                        m_trajectory_j.goal.Ref(4, 3).Assign(m_tool_engage_upper_position);
                    } else {
                        m_trajectory_j.goal.Ref(4, 3).Assign(m_tool_engage_lower_position);
                    }
                } else {
                    m_trajectory_j.goal.Ref(4, 3).SetAll(0.0); // back to zero position
                }
                m_trajectory_j.end_time = 0.0;
                std::stringstream message;
                message << this->GetName() << ": engaging tool " << EngagingStage - 1 << " of " << LastEngagingStage - 1;
                m_arm_interface->SendStatus(message.str());
                EngagingStage++;
            }
        }
        break;

    default:
        m_arm_interface->SendError(this->GetName() + " error while evaluating trajectory.");
        SetDesiredState("FAULT");
        break;
    }
}

void mtsIntuitiveResearchKitPSM::EnterToolEngaged(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);
    Tool.IsEngaged = true;
    set_LED_pattern(mtsIntuitiveResearchKit::Green200,
                    mtsIntuitiveResearchKit::Green200,
                    false, false);

    // restore default PID tracking error
    PID.set_measured_setpoint_tolerance(PID.measured_setpoint_tolerance);
    // resize kinematics vectors
    cmnDataCopy(m_kin_measured_js.Name(), m_configuration_js.Name());
    m_kin_measured_js.Position().SetSize(number_of_joints_kinematics());
    m_kin_measured_js.Velocity().SetSize(number_of_joints_kinematics());
    m_kin_measured_js.Effort().SetSize(number_of_joints_kinematics());
    cmnDataCopy(m_kin_setpoint_js.Name(), m_configuration_js.Name());
    m_kin_setpoint_js.Position().SetSize(number_of_joints_kinematics());
    m_kin_setpoint_js.Velocity().SetSize(number_of_joints_kinematics());
    m_kin_setpoint_js.Effort().SetSize(number_of_joints_kinematics());
    // jaw
    cmnDataCopy(m_jaw_measured_js.Name(), m_jaw_configuration_js.Name());
    m_jaw_measured_js.Position().SetSize(1);
    m_jaw_measured_js.Velocity().SetSize(1);
    m_jaw_measured_js.Effort().SetSize(1);
    cmnDataCopy(m_jaw_setpoint_js.Name(), m_jaw_configuration_js.Name());
    m_jaw_setpoint_js.Position().SetSize(1);
    m_jaw_setpoint_js.Velocity().SetSize(0);
    m_jaw_setpoint_js.Effort().SetSize(1);
}

void mtsIntuitiveResearchKitPSM::TransitionToolEngaged(void)
{
    Tool.NeedEngage = false;
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("HOMED");
    }
}

void mtsIntuitiveResearchKitPSM::EnterManual(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    m_jaw_servo_jf = 0.0;
    free();
}

void mtsIntuitiveResearchKitPSM::RunManual(void)
{
    if (mControlCallback) {
        mControlCallback->Execute();
    }
}

void mtsIntuitiveResearchKitPSM::LeaveManual(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);
    hold();
}

double mtsIntuitiveResearchKitPSM::clip_jaw_jp(double jp)
{
    if (jp > m_jaw_configuration_js.PositionMax().at(0)) {
        jp = m_jaw_configuration_js.PositionMax().at(0);
    } else if (jp < m_jaw_configuration_js.PositionMin().at(0)) {
        jp = m_jaw_configuration_js.PositionMin().at(0);
    }

    return jp;
}

void mtsIntuitiveResearchKitPSM::jaw_servo_jp(const prmPositionJointSet & jawPosition)
{
    // we need to need to at least ready to control in cartesian space
    if (!ArmIsReady("jaw_servo_jp", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (m_control_space) {
    case mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE:
        if (! (m_control_mode == mtsIntuitiveResearchKitControlTypes::POSITION_MODE)) {
            SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitControlTypes::POSITION_MODE);
            // make sure all other joints have a reasonable cartesian
            // goal for all other joints
            m_servo_cs.Position().Assign(m_setpoint_cp.Position());
            m_servo_cs.PositionIsValid() = true;
            m_servo_cs.VelocityIsValid() = false;
            m_servo_cs.ForceIsValid() = false;
        }
        break;
    default:
        if (! (m_control_mode == mtsIntuitiveResearchKitControlTypes::POSITION_MODE)) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitControlTypes::POSITION_MODE);
            // make sure all other joints have a reasonable goal
            m_servo_jp.Assign(m_pid_setpoint_js.Position(), number_of_joints());
        }
    }

    // save goal
    m_jaw_servo_jp = jawPosition.Goal().at(0);
    m_servo_jp.at(6) = m_jaw_servo_jp;
    m_pid_new_goal = true;
}

void mtsIntuitiveResearchKitPSM::jaw_move_jp(const prmPositionJointSet & jawPosition)
{
    // we need to need to at least ready to control in cartesian space
    if (!ArmIsReady("jaw_move_jp", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (m_control_space) {
    case mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE:
        if (m_control_mode != mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE);
            // make sure all other joints have a reasonable goal
            m_trajectory_j.goal.Assign(m_pid_setpoint_js.Position(), number_of_joints_kinematics());
        }
        break;
    default:
        if (m_control_mode != mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE);
            // make sure all other joints have a reasonable goal
            m_trajectory_j.goal.Assign(m_pid_setpoint_js.Position());
        }
    }

    // force trajectory re-evaluation with new goal for last joint
    control_move_jp_on_start();
    m_jaw_servo_jp = jawPosition.Goal().at(0);
    m_trajectory_j.goal[6] = m_jaw_servo_jp;
}

void mtsIntuitiveResearchKitPSM::servo_jp_internal(const vctDoubleVec & jp,
                                                   const vctDoubleVec & jv)
{
    if (!is_cartesian_ready()) {
        mtsIntuitiveResearchKitArm::servo_jp_internal(jp, jv);
        return;
    }

    CMN_ASSERT(m_servo_jp_param.Goal().size() == 7);
    // first 6 joints, assign positions and check limits
    vctDoubleVec jp_clipped(jp.Ref(number_of_joints_kinematics()));
    clip_jp(jp_clipped);
    ToJointsPID(jp_clipped, m_servo_jp_param.Goal());

    if (jp.size() == 7) {
        m_jaw_servo_jp = clip_jaw_jp(jp.at(6));
    }

    // velocity - current code only support jaw_servo_jv if servo_jp has a velocity goal
    const size_t jv_size = jv.size();
    m_servo_jp_param.Velocity().SetSize(7);
    if (jv_size != 0) {
        ToJointsPID(jv, m_servo_jp_param.Velocity());
    }
    m_servo_jp_param.Goal().at(6) = m_jaw_servo_jp;
    if (jv_size != 0) {
        m_servo_jp_param.Velocity().at(6) = 0.0;
    }
    m_servo_jp_param.SetTimestamp(StateTable.GetTic());
    if (m_has_coupling) {
        m_servo_jp_param.Goal() = m_coupling.JointToActuatorPosition() * m_servo_jp_param.Goal();
        if (jv_size != 0) {
            m_servo_jp_param.Velocity() = m_coupling.JointToActuatorPosition() * m_servo_jp_param.Velocity();
        }
    }
    PID.servo_jp(m_servo_jp_param);

    apply_feed_forward();
}

void mtsIntuitiveResearchKitPSM::jaw_servo_jf(const prmForceTorqueJointSet & effort)
{
    if (!ArmIsReady("servo_jf", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (m_control_space) {
    case mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE:
        if (m_control_mode != mtsIntuitiveResearchKitControlTypes::EFFORT_MODE) {
            SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitControlTypes::EFFORT_MODE);
            // make sure all other joints have a reasonable cartesian
            // goal
            m_servo_cf.Force().SetAll(0.0);
        }
        break;
    default:
        // we are initiating the control mode switch
        SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                               mtsIntuitiveResearchKitControlTypes::EFFORT_MODE);
        // make sure all other joints have a reasonable goal
        m_servo_jf.ForceTorque().SetAll(0.0);
    }

    // save the desired effort
    m_jaw_servo_jf = effort.ForceTorque().at(0);
}

void mtsIntuitiveResearchKitPSM::servo_jf_internal(const vctDoubleVec & newEffort)
{

    // pad array for PID
    vctDoubleVec torqueDesired(number_of_joints(), 0.0); // for PID
    if (m_snake_like) {
        std::cerr << CMN_LOG_DETAILS << " need to convert 8 joints from snake to 6 for force control" << std::endl;
    } else {
        torqueDesired.Assign(newEffort, number_of_joints_kinematics());
    }
    // add torque for jaws
    torqueDesired.at(6) = m_jaw_servo_jf;

    if (!is_cartesian_ready()) {
        // set all tool joints to have zero effort
        torqueDesired.Ref(torqueDesired.size() - 3, 3).Zeros();
    }

    // convert to cisstParameterTypes
    m_servo_jf_param.SetForceTorque(torqueDesired);
    m_servo_jf_param.SetTimestamp(StateTable.GetTic());
    if (m_has_coupling) {
        m_servo_jf_param.ForceTorque() = m_coupling.JointToActuatorEffort() * m_servo_jf_param.ForceTorque();
    }
    PID.servo_jf(m_servo_jf_param);

    apply_feed_forward();
}

void mtsIntuitiveResearchKitPSM::feed_forward_jf_internal(const vctDoubleVec & jf)
{
    if (!is_cartesian_ready()) {
        mtsIntuitiveResearchKitArm::feed_forward_jf_internal(jf);
        return;
    }

    // pad array for PID
    vctDoubleVec joint_efforts(number_of_joints(), 0.0); // for PID
    joint_efforts.at(6) = m_jaw_servo_jf;

    if (m_snake_like) {
        std::cerr << CMN_LOG_DETAILS << " need to convert 8 joints from snake to 6 for force control" << std::endl;
    } else if (jf.size() > 0) {
        joint_efforts.Ref(number_of_joints_kinematics()).Assign(jf);
    }

    m_feed_forward_jf_param.ForceTorque().Zeros();

    if (m_has_coupling) {
        m_feed_forward_jf_param.ForceTorque().Add(m_coupling.JointToActuatorEffort() * joint_efforts);
    } else {
        m_feed_forward_jf_param.ForceTorque().Add(joint_efforts);
    }

    // convert to cisstParameterTypes
    m_servo_jf_param.SetForceTorque(m_feed_forward_jf_param.ForceTorque());
    m_servo_jf_param.SetTimestamp(StateTable.GetTic());
    m_feed_forward_jf_param.SetTimestamp(StateTable.GetTic());
    PID.feed_forward_servo_jf(m_feed_forward_jf_param);
}

void mtsIntuitiveResearchKitPSM::control_move_jp_on_stop(const bool goal_reached)
{
    if (is_cartesian_ready()) {
        // save end position as starting servo for jaws
        m_jaw_servo_jp = m_servo_jp_param.Goal().at(6);
    }
    mtsIntuitiveResearchKitArm::control_move_jp_on_stop(goal_reached);
}

void mtsIntuitiveResearchKitPSM::set_adapter_present(const bool & present)
{
    Adapter.IsEngaged = false;
    Adapter.IsPresent = present;
    if (present) {
        // we will need to engage this adapter
        Adapter.NeedEngage = true;
        m_arm_interface->SendStatus(this->GetName() + ": adapter detected");
    } else {
        Adapter.NeedEngage = false;
        m_arm_interface->SendStatus(this->GetName() + ": no adapter detected");
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerAdapter(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        set_adapter_present(true);
        break;
    case prmEventButton::RELEASED:
        set_adapter_present(false);
        // detect if adapeter is removed while engaging
        if (mArmState.CurrentState() == "ENGAGING_ADAPTER") {
            mArmState.SetCurrentState("HOMED");
        }
        break;
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitPSM::set_tool_present_and_configured(const bool & present,
                                                                 const bool & configured)
{
    // no change
    if ((present == m_tool_present) && (configured == m_tool_configured)) {
        return;
    }
    // tool present change
    if (present != m_tool_present) {
        m_tool_present = present;
        if (m_tool_present) {
            m_arm_interface->SendStatus(this->GetName() + ": tool detected");
        } else {
            // assumes this is called when tool is not present anymore
            ToolEvents.tool_type(std::string());
            m_arm_interface->SendStatus(this->GetName() + ": no tool detected");
        }
    }
    // tool configured change
    if (configured != m_tool_configured) {
        m_tool_configured = configured;
        if (m_tool_configured) {
        } else {
            // remove tool tip offset
            Manipulator->DeleteTools();
            // in any case, we just need the first 3 links
            Manipulator->Truncate(3);
            // and no coupling
            m_has_coupling = false;
        }
    }
    // update for users
    update_configuration_js();
    // refresh data to take coupling into account
    get_robot_data();

    // general update
    if (m_tool_present && m_tool_configured) {
        // we will need to engage this tool
        Tool.NeedEngage = true;
        ToolEvents.tool_type(m_tool_list.Name(m_tool_index));
    }
}

void mtsIntuitiveResearchKitPSM::emulate_tool_present(const bool & present)
{
    Tool.IsEmulated = present;
    prmEventButton emulatedEvent;
    if (present) {
        emulatedEvent.Type() = prmEventButton::PRESSED;
    } else {
        emulatedEvent.Type() = prmEventButton::RELEASED;
    }
    EventHandlerTool(emulatedEvent);
}

void mtsIntuitiveResearchKitPSM::set_tool_type(const std::string & toolType)
{
    if (m_tool_type_requested || m_simulated) {
        EventHandlerToolType(toolType);
        if (m_tool_configured) {
            m_tool_type_requested = false;
        }
    } else {
        m_arm_interface->SendWarning(this->GetName() + ": received request to set tool type but not expecting it now.  Request ignored.");
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerTool(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        // if the adapter was engaging, make sure it stops immediately
        if (mArmState.CurrentState() == "ENGAGING_ADAPTER") {
            mArmState.SetCurrentState("HOMED");
        }
        // then figure out which tool we're using
        switch (m_tool_detection) {
        case mtsIntuitiveResearchKitToolTypes::AUTOMATIC:
            set_tool_present_and_configured(true, false);
            Dallas.TriggerRead();
            break;
        case mtsIntuitiveResearchKitToolTypes::MANUAL:
            set_tool_present_and_configured(true, false);
            if (is_joint_ready()) {
                m_tool_type_requested = true;
                ToolEvents.tool_type_request();
                m_arm_interface->SendWarning(this->GetName() + ": tool type requested from user");
            }
            break;
        case mtsIntuitiveResearchKitToolTypes::FIXED:
            {
                const std::string toolFile = m_tool_list.File(m_tool_index);
                m_arm_interface->SendStatus(this->GetName() + ": using tool file \"" + toolFile
                                            + "\" for fixed tool: " + m_tool_list.FullDescription(m_tool_index));
                bool tool_configured = ConfigureTool(toolFile);
                if (!tool_configured) {
                    m_arm_interface->SendError(this->GetName() + ": failed to configure fixed tool, check terminal output and cisstLog file");
                    ToolEvents.tool_type(std::string("ERROR"));
                }
                set_tool_present_and_configured(true, tool_configured);
            }
            break;
        default:
            break;
        }
        break;
    case prmEventButton::RELEASED:
        switch (m_tool_detection) {
        case mtsIntuitiveResearchKitToolTypes::AUTOMATIC:
        case mtsIntuitiveResearchKitToolTypes::MANUAL:
        case mtsIntuitiveResearchKitToolTypes::FIXED:
            Manipulator->Truncate(3);
            set_tool_present_and_configured(false, false);
            break;
        default:
            break;
        }
        // detect if tool is removed while engaging
        if (mArmState.CurrentState() == "ENGAGING_TOOL") {
            mArmState.SetCurrentState("HOMED");
        }
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerManipClutch(const prmEventButton & button)
{
    // Pass events
    ClutchEvents.ManipClutch(button);

    // Start manual mode but save the previous state
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        if (ArmIsReady("Clutch", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
            ClutchEvents.ManipClutchPreviousState = mArmState.CurrentState();
            PID.enabled(ClutchEvents.PIDEnabledPreviousState);
            mArmState.SetCurrentState("MANUAL");
            set_LED_pattern(mtsIntuitiveResearchKit::Blue200,
                            mtsIntuitiveResearchKit::Green200,
                            true, false);
        } else {
            m_arm_interface->SendWarning(this->GetName() + ": arm not ready yet, manipulator clutch ignored");
        }
        break;
    case prmEventButton::RELEASED:
        if (mArmState.CurrentState() == "MANUAL") {
            // go back to state before clutching
            mArmState.SetCurrentState(ClutchEvents.ManipClutchPreviousState);
            PID.enable(ClutchEvents.PIDEnabledPreviousState);
        }
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch(const prmEventButton & button)
{
    bool value = (button.Type() == prmEventButton::PRESSED);
    if (value
        && (m_operating_state.State() != prmOperatingState::ENABLED)) {
        m_arm_interface->SendWarning(this->GetName() + ": arm needs to be enabled to release the SUJ brakes");
    } else {
        SUJClutch.Brake(value);
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerToolType(const std::string & toolType)
{
    m_arm_interface->SendStatus(this->GetName() + ": setting up for tool type \"" + toolType + "\"");
    // check if the tool is in the supported list
    if (!m_tool_list.Find(toolType, m_tool_index)) {
        CMN_LOG_CLASS_RUN_ERROR << "Supported tool types are:\n" << m_tool_list.PossibleNames("\n") << std::endl;
        m_arm_interface->SendError(this->GetName() + ": tool type \""
                                   + toolType + "\" is not supported, see cisstLog for details");
        ToolEvents.tool_type(std::string("ERROR"));
        return;
    }
    // supported tools
    const std::string toolFile = m_tool_list.File(m_tool_index);
    m_arm_interface->SendStatus(this->GetName() + ": using tool file \"" + toolFile
                                + "\" for: " + m_tool_list.FullDescription(m_tool_index));
    bool tool_configured = ConfigureTool(toolFile);
    if (!tool_configured) {
        m_arm_interface->SendError(this->GetName() + ": failed to configure tool \"" + toolType + "\", check terminal output and cisstLog file");
        ToolEvents.tool_type(std::string("ERROR"));
    }
    set_tool_present_and_configured(m_tool_present, tool_configured);
}

/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitConsole_h
#define _mtsIntuitiveResearchKitConsole_h

#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstMultiTask/mtsDelayedConnections.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmKeyValue.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>
#include <sawIntuitiveResearchKit/console_configuration_t.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// for ROS console
namespace dvrk {
    class console;
}

class mtsTextToSpeech;
class mtsDaVinciEndoscopeFocus;
class mtsIntuitiveResearchKitArm;

class CISST_EXPORT mtsIntuitiveResearchKitConsole: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    friend class mtsIntuitiveResearchKitConsoleQt;
    friend class dvrk::console;

    class CISST_EXPORT arm_proxy_t {
     public:

        friend class mtsIntuitiveResearchKitConsole;

        arm_proxy_t(const std::string & name, mtsIntuitiveResearchKitConsole * console);

        NOT_COPYABLE(arm_proxy_t);
        NOT_MOVEABLE(arm_proxy_t);

        /*! Load and validate configuration from json */
        void configure(const Json::Value & json_config);

        /*! Create and configure the robot arm. */
        void create_arm(void);

        /*! Create a new PID component and connect it to the proper RobotIO
          interface.  If the period in seconds is zero, the PID will be tied to
          IO using the ExecIn/ExecOut interfaces. */
        void create_PID(void);

        void create_IO(void);

        /*! Check if mBaseFrame has a valid name and if it does
          set_base_frame on the arm. */
        void set_base_frame_if_needed(void);

        /*! Connect all interfaces specific to this arm. */
        bool Connect(void);

        /*! Accessors */

        dvrk::generation_t generation(void) const;

        std::string m_name;
        mtsIntuitiveResearchKitConsole * m_console = nullptr;
        dvrk::arm_proxy_configuration_t m_config;

        std::shared_ptr<mtsIntuitiveResearchKitArm> m_arm = nullptr;

        std::string m_serial;
        bool m_calibration_mode = false;

        // low level
        std::string m_IO_component_name;
        std::string m_IO_configuration_file;
        std::string m_IO_gripper_configuration_file; // for MTMs only
        // PID
        std::string m_PID_component_name;
        std::string m_PID_configuration_file; // actual file
        // arm
        std::string m_arm_configuration_file;
        std::string m_arm_component_name; // for generic or derived
        std::string m_arm_interface_name; // for generic or derived

        double m_arm_period = mtsIntuitiveResearchKit::ArmPeriod;
        // add ROS bridge
        bool m_skip_ROS_bridge;

    protected:
        // base frame
        // (name and frame) OR (component and interface)
        prmPositionCartesianSet m_base_frame;
        std::string m_base_frame_component_name;
        std::string m_base_frame_interface_name;

        mtsFunctionWrite state_command;
        mtsFunctionVoid hold;

        // interfaces used to communicate with components created for a given arm
        mtsInterfaceRequired * m_IO_interface_required = nullptr;
        mtsInterfaceRequired * m_IO_dallas_interface_required = nullptr;
        mtsInterfaceRequired * m_PID_interface_required = nullptr;
        mtsInterfaceRequired * m_arm_interface_required = nullptr;

        // this is used only by PSMs and ECM
        mtsInterfaceRequired * SUJInterfaceRequiredFromIO = nullptr;
        mtsInterfaceRequired * SUJInterfaceRequiredFromIO2 = nullptr; // for Si second clutch button
        mtsInterfaceRequired * SUJInterfaceRequiredToSUJ = nullptr;
        mtsFunctionWrite SUJClutch;
        bool m_SUJ_clutched = false;

        void SUJClutchEventHandlerFromIO(const prmEventButton & button) {
            if (button.Type() == prmEventButton::PRESSED) {
                m_SUJ_clutched = true;
                SUJClutch(true);
            } else {
                m_SUJ_clutched = false;
                SUJClutch(false);
            }
        }

        void CurrentStateEventHandler(const prmOperatingState & currentState);
    };

    class CISST_EXPORT TeleopECM {
    public:
        typedef enum {TELEOP_ECM, TELEOP_ECM_DERIVED, TELEOP_ECM_GENERIC} TeleopECMType;
        friend class mtsIntuitiveResearchKitConsole;
        friend class mtsIntuitiveResearchKitConsoleQt;
        friend class dvrk::console;

        TeleopECM(const std::string & name);

        NOT_COPYABLE(TeleopECM);
        NOT_MOVEABLE(TeleopECM);

        /*! Create and configure the robot arm. */
        void ConfigureTeleop(const TeleopECMType type,
                             const double & period_in_seconds,
                             const Json::Value & jsonConfig);

    protected:
        std::string m_name;
        TeleopECMType m_type;
        mtsFunctionWrite state_command;
        mtsInterfaceRequired * InterfaceRequired;
    };

    class CISST_EXPORT teleop_PSM_proxy_t {
    public:

        friend class mtsIntuitiveResearchKitConsole;

        std::string m_name;
        mtsIntuitiveResearchKitConsole * m_console = nullptr;
        dvrk::teleop_PSM_proxy_configuration_t m_config;

        teleop_PSM_proxy_t(const std::string & name, mtsIntuitiveResearchKitConsole * console);

        NOT_COPYABLE(teleop_PSM_proxy_t);
        NOT_MOVEABLE(teleop_PSM_proxy_t);

        /*! Create and configure the robot arm. */
        void configure(const Json::Value & json_config);

        /*! Create and configure the teleoperation component. */
        void create_teleop(void);

        /*! Turn on/off selected */
        inline const bool & selected(void) const {
            return m_selected;
        }
        inline void set_selected(const bool select) {
            m_selected = select;
        }

    protected:
        bool m_selected;
        mtsFunctionWrite state_command;
        mtsFunctionWrite set_scale;
        mtsInterfaceRequired * m_interface_required;
    };



    
    mtsIntuitiveResearchKitConsole(const std::string & componentName);
    inline virtual ~mtsIntuitiveResearchKitConsole() {}

    /*! Tells the application to run in calibration mode, i.e. turn
      off all checks using potentiometers and force encoder re-bias
      from potentiometers when the application starts.  This method
      must be called before Configure. */
    void set_calibration_mode(const bool mode);
    const bool & calibration_mode(void) const;
    void calibration_mode(bool & result) const;

    /*! Configure console using JSON file. To test is the configuration
      succeeded, used method Configured().
    */
    void Configure(const std::string & filename);

    /*! Method to check if the configuration was successful, ideally called
      after a call to Configure.
    */
    const bool & Configured(void) const;

    void Startup(void);
    void Run(void);
    void Cleanup(void);

    std::string GetArmIOComponentName(const std::string & arm_name);

    void AddFootpedalInterfaces(void);

    bool Connect(void);

    std::string find_file(const std::string & filename) const;

 protected:
    bool m_configured;
    cmnPath m_config_path;
    mtsDelayedConnections m_connections;

    double mTimeOfLastErrorBeep;
    bool mTeleopEnabled = false;
    bool mTeleopDesired = false;
    bool mTeleopPSMRunning = false;
    bool mTeleopECMRunning = false;
    bool mTeleopEnabledBeforeCamera;

    typedef std::map<std::string, std::shared_ptr<arm_proxy_t>> arm_proxies_t;
    arm_proxies_t m_arm_proxies;

    /*! Pointer to mtsTextToSpeech component */
    std::shared_ptr<mtsTextToSpeech> m_text_to_speech;

    /*! List to manage multiple PSM teleoperations */
    typedef std::map<std::string, std::shared_ptr<teleop_PSM_proxy_t>> teleop_PSM_proxies_t;
    teleop_PSM_proxies_t m_teleop_PSM_proxies;

    /*! List to manage the teleopPSM components for each PSM and MTM */
    typedef std::multimap<std::string, std::shared_ptr<teleop_PSM_proxy_t>> teleop_PSM_proxies_by_arm_t;
    teleop_PSM_proxies_by_arm_t m_teleop_PSM_proxies_by_psm;
    teleop_PSM_proxies_by_arm_t m_teleop_PSM_proxies_by_MTM;

    /*! Name of default MTM to cycle teleops if no name is provided */
    std::string mTeleopMTMToCycle;

    /*! Single ECM bimanual teleoperation */
    TeleopECM * mTeleopECM = nullptr;

    /*! Head sensor */
    mtsComponent * mHeadSensor = nullptr;

    /*! daVinci Endoscope Focus */
    mtsDaVinciEndoscopeFocus * mDaVinciEndoscopeFocus = nullptr;

    bool add_arm_interfaces(std::shared_ptr<arm_proxy_t> arm_proxy);

    // these two methods have exact same implementation.it would be
    // nice to have a base class, or template this
    bool AddTeleopECMInterfaces(TeleopECM * teleop);
    bool add_teleop_PSM_interfaces(std::shared_ptr<teleop_PSM_proxy_t> teleop_proxy);

    bool ConfigureECMTeleopJSON(const Json::Value & jsonTeleop);
    bool ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop);

    void power_off(void);
    void power_on(void);
    void home(void);
    void DisableFaultyArms(void);
    void teleop_enable(const bool & enable);
    void cycle_teleop_PSM_by_MTM(const std::string & mtmName);
    void select_teleop_PSM(const prmKeyValue & mtmPsm);
    bool GetPSMSelectedForMTM(const std::string & mtmName, std::string & psmName) const;
    bool GetMTMSelectedForPSM(const std::string & psmName, std::string & mtmName) const;
    void EventSelectedTeleopPSMs(void) const;
    void UpdateTeleopState(void);
    void set_scale(const double & scale);
    void set_volume(const double & volume);
    void beep(const vctDoubleVec & values); // duration, frequency, volume
    void string_to_speech(const std::string & text);
    bool mHasIO = false;
    std::unique_ptr<arm_proxy_t> m_SUJ = nullptr;
    void ClutchEventHandler(const prmEventButton & button);
    void CameraEventHandler(const prmEventButton & button);
    void OperatorPresentEventHandler(const prmEventButton & button);

    void ConnectInternal(bool &ret) const;

    bool m_calibration_mode = false;

    struct {
        mtsFunctionWrite beep;
        mtsFunctionWrite string_to_speech;
        mtsFunctionWrite volume; // event
    } audio;
    double m_audio_volume;
    bool mChatty;

    struct {
        mtsFunctionWrite clutch;
        mtsFunctionWrite camera;
        mtsFunctionWrite operator_present;
        mtsFunctionWrite teleop_enabled;
    } console_events;
    bool mOperatorPresent;
    bool mCameraPressed;

    std::string m_IO_component_name; // for actuator IOs

    // components used for events (digital inputs)
    typedef std::pair<std::string, std::string> InterfaceComponentType;
    typedef std::map<std::string, InterfaceComponentType> DInputSourceType;
    DInputSourceType mDInputSources;

    mtsInterfaceRequired * m_IO_interface = nullptr;
    struct {
        mtsFunctionVoid close_all_relays;
    } IO;
    bool m_close_all_relays_from_config = true;

    mtsInterfaceProvided * mInterface = nullptr;
    struct {
        mtsFunctionWrite ArmCurrentState;
        mtsFunctionWrite scale;
        mtsFunctionWrite teleop_PSM_selected;
        mtsFunctionWrite teleop_PSM_unselected;
    } ConfigurationEvents;

    void ErrorEventHandler(const mtsMessage & message);
    void WarningEventHandler(const mtsMessage & message);
    void StatusEventHandler(const mtsMessage & message);

    std::map<std::string, prmOperatingState> ArmStates;
    void SetArmCurrentState(const std::string & armName,
                            const prmOperatingState & currentState);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitConsole);

#endif // _mtsIntuitiveResearchKitConsole_h

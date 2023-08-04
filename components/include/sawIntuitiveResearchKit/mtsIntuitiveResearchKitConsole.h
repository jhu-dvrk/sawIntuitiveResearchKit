/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

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

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// for ROS console
namespace dvrk {
    class console;
}

class mtsTextToSpeech;
class mtsDaVinciHeadSensor;
class mtsDaVinciEndoscopeFocus;
class mtsIntuitiveResearchKitArm;

class CISST_EXPORT mtsIntuitiveResearchKitConsole: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

 public:
    friend class mtsIntuitiveResearchKitConsoleQt;
    friend class dvrk::console;

    class CISST_EXPORT Arm {
    public:
        typedef enum {ARM_UNDEFINED,
                      ARM_MTM, ARM_PSM, ARM_ECM,
                      ARM_SUJ_Classic, ARM_SUJ_Si, ARM_SUJ_Fixed,
                      ARM_MTM_GENERIC, ARM_PSM_GENERIC, ARM_ECM_GENERIC,
                      ARM_MTM_DERIVED, ARM_PSM_DERIVED, ARM_ECM_DERIVED,
                      ARM_PSM_SOCKET,
                      FOCUS_CONTROLLER} ArmType;

        typedef enum {SIMULATION_NONE,
                      SIMULATION_KINEMATIC,
                      SIMULATION_DYNAMIC} SimulationType;

        friend class mtsIntuitiveResearchKitConsole;
        friend class mtsIntuitiveResearchKitConsoleQt;
        friend class dvrk::console;

        bool generic(void) const;
        bool psm(void) const;
        bool mtm(void) const;
        bool ecm(void) const;
        bool suj(void) const;
        bool native_or_derived(void) const;
        bool native_or_derived_mtm(void) const;
        bool native_or_derived_psm(void) const;
        bool native_or_derived_ecm(void) const;
        void set_generation(const mtsIntuitiveResearchKitArm::GenerationType generation);
        mtsIntuitiveResearchKitArm::GenerationType generation(void) const;
        bool expects_PID(void) const;
        bool expects_IO(void) const;

        Arm(mtsIntuitiveResearchKitConsole * console,
            const std::string & name,
            const std::string & ioComponentName);

        /*! Create a new PID component and connect it to the proper RobotIO
          interface.  If the period in seconds is zero, the PID will be tied to
          IO using the ExecIn/ExecOut interfaces. */
        void ConfigurePID(const std::string & configFile,
                          const double & periodInSeconds = 0.0 * cmn_ms);

        /*! Create and configure the robot arm. */
        void ConfigureArm(const ArmType armType,
                          const std::string & kinematicsConfigFile,
                          const double & periodInSeconds = mtsIntuitiveResearchKit::ArmPeriod);

        /*! Check if mBaseFrame has a valid name and if it does
          set_base_frame on the arm. */
        void SetBaseFrameIfNeeded(mtsIntuitiveResearchKitArm * armPointer);

        /*! Connect all interfaces specific to this arm. */
        bool Connect(void);

        /*! Accessors */
        const std::string & Name(void) const;
        const std::string & SocketComponentName(void) const;
        const std::string & ComponentName(void) const;
        const std::string & InterfaceName(void) const;
        const std::string & IOComponentName(void) const;
        const std::string & PIDComponentName(void) const;

    protected:
        mtsIntuitiveResearchKitConsole * m_console = nullptr;
        std::string m_name;
        ArmType m_type;
        mtsIntuitiveResearchKitArm::GenerationType m_generation = mtsIntuitiveResearchKitArm::GENERATION_UNDEFINED;
        std::string m_serial;
        SimulationType m_simulation;
        bool m_calibration_mode = false;
        cmnPath m_config_path;

        // low level
        std::string m_IO_component_name;
        std::string m_IO_configuration_file;
        std::string m_IO_gripper_configuration_file; // for MTMs only
        // PID
        std::string m_PID_component_name;
        std::string m_PID_configuration_file;
        // arm
        std::string m_arm_component_name;
        std::string m_arm_interface_name;
        std::string m_arm_configuration_file;
        double m_arm_period;
        // socket
        std::string m_IP;
        int m_port;
        bool m_socket_server;
        std::string m_socket_component_name;
        // add ROS bridge
        bool m_skip_ROS_bridge;

        // base frame
        // (name and frame) OR (component and interface)
        prmPositionCartesianSet m_base_frame;
        std::string m_base_frame_component_name;
        std::string m_base_frame_interface_name;

        mtsFunctionWrite state_command;
        mtsFunctionVoid hold;
        mtsInterfaceRequired * IOInterfaceRequired = nullptr;
        mtsInterfaceRequired * IODallasInterfaceRequired = nullptr;
        mtsInterfaceRequired * PIDInterfaceRequired = nullptr;
        mtsInterfaceRequired * ArmInterfaceRequired = nullptr;

        // this is used only by PSMs and ECM
        mtsInterfaceRequired * SUJInterfaceRequiredFromIO = nullptr;
        mtsInterfaceRequired * SUJInterfaceRequiredFromIO2 = nullptr; // for Si second clutch button
        mtsInterfaceRequired * SUJInterfaceRequiredToSUJ;
        mtsFunctionWrite SUJClutch;
        bool mSUJClutched;

        void SUJClutchEventHandlerFromIO(const prmEventButton & button) {
            if (button.Type() == prmEventButton::PRESSED) {
                mSUJClutched = true;
                SUJClutch(true);
            } else {
                mSUJClutched = false;
                SUJClutch(false);
            }
        }

        void CurrentStateEventHandler(const prmOperatingState & currentState);
        prmOperatingState m_operating_state;
    };

    class CISST_EXPORT TeleopECM {
    public:
        typedef enum {TELEOP_ECM, TELEOP_ECM_DERIVED, TELEOP_ECM_GENERIC} TeleopECMType;
        friend class mtsIntuitiveResearchKitConsole;
        friend class mtsIntuitiveResearchKitConsoleQt;
        friend class dvrk::console;

        TeleopECM(const std::string & name);

        /*! Create and configure the robot arm. */
        void ConfigureTeleop(const TeleopECMType type,
                             const double & periodInSeconds,
                             const Json::Value & jsonConfig);

        /*! Accessors */
        const std::string & Name(void) const;

    protected:
        std::string m_name;
        TeleopECMType m_type;
        mtsFunctionWrite state_command;
        mtsInterfaceRequired * InterfaceRequired;
    };

    class TeleopPSM {
    public:
        typedef enum {TELEOP_PSM, TELEOP_PSM_DERIVED, TELEOP_PSM_GENERIC} TeleopPSMType;
        friend class mtsIntuitiveResearchKitConsole;
        friend class mtsIntuitiveResearchKitConsoleQt;
        friend class dvrk::console;

        TeleopPSM(const std::string & name,
                  const std::string & nameMTM,
                  const std::string & namePSM);

        /*! Create and configure the robot arm. */
        void ConfigureTeleop(const TeleopPSMType type,
                             const double & periodInSeconds,
                             const Json::Value & jsonConfig);

        /*! Accessors */
        const std::string & Name(void) const;

        /*! Turn on/off selected */
        inline const bool & Selected(void) const {
            return mSelected;
        }
        inline void SetSelected(const bool selected) {
            mSelected = selected;
        }

    protected:
        bool mSelected;
        std::string m_name;
        TeleopPSMType m_type;
        std::string mMTMName;
        std::string mPSMName;
        mtsFunctionWrite state_command;
        mtsFunctionWrite set_scale;
        mtsInterfaceRequired * InterfaceRequired;
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

    bool AddArm(Arm * newArm);
    bool AddArm(mtsComponent * genericArm, const Arm::ArmType armType);
    std::string GetArmIOComponentName(const std::string & armName);

    void AddFootpedalInterfaces(void);

    bool Connect(void);

    std::string locate_file(const std::string & filename);

 protected:
    bool m_configured;
    cmnPath m_config_path;
    mtsDelayedConnections mConnections;

    double mTimeOfLastErrorBeep;
    bool mTeleopEnabled = false;
    bool mTeleopDesired = false;
    bool mTeleopPSMRunning = false;
    bool mTeleopECMRunning = false;
    bool mTeleopEnabledBeforeCamera;

    typedef std::map<std::string, Arm *> ArmList;
    ArmList mArms;

    /*! Pointer to mtsTextToSpeech component */
    mtsTextToSpeech * mTextToSpeech;

    /*! List to manage multiple PSM teleoperations */
    typedef std::map<std::string, TeleopPSM *> TeleopPSMList;
    TeleopPSMList mTeleopsPSM;

    /*! List to manage the teleopPSM components for each PSM */
    typedef std::multimap<std::string, TeleopPSM *> TeleopPSMByPSMList;
    TeleopPSMByPSMList mTeleopsPSMByPSM;

    /*! List to manage the teleopPSM components for each MTM */
    typedef std::multimap<std::string, TeleopPSM *> TeleopPSMByMTMList;
    TeleopPSMByMTMList mTeleopsPSMByMTM;

    /*! Name of default MTM to cycle teleops if no name is provided */
    std::string mTeleopMTMToCycle;

    /*! Single ECM bimanual teleoperation */
    TeleopECM * mTeleopECM;

    /*! daVinci Head Sensor */
    mtsDaVinciHeadSensor * mDaVinciHeadSensor;

    /*! daVinci Endoscope Focus */
    mtsDaVinciEndoscopeFocus * mDaVinciEndoscopeFocus;

    /*! Find all arm data from JSON configuration. */
    bool ConfigureArmJSON(const Json::Value & jsonArm,
                          const std::string & ioComponentName);
    bool AddArmInterfaces(Arm * arm);

    // these two methods have exact same implementation.it would be
    // nice to have a base class, or template this
    bool AddTeleopECMInterfaces(TeleopECM * teleop);
    bool AddTeleopPSMInterfaces(TeleopPSM * teleop);

    bool ConfigureECMTeleopJSON(const Json::Value & jsonTeleop);
    bool ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop);

    void power_off(void);
    void power_on(void);
    void home(void);
    void DisableFaultyArms(void);
    void teleop_enable(const bool & enable);
    void cycle_teleop_psm_by_mtm(const std::string & mtmName);
    void select_teleop_psm(const prmKeyValue & mtmPsm);
    bool GetPSMSelectedForMTM(const std::string & mtmName, std::string & psmName) const;
    bool GetMTMSelectedForPSM(const std::string & psmName, std::string & mtmName) const;
    void EventSelectedTeleopPSMs(void) const;
    void UpdateTeleopState(void);
    void set_scale(const double & scale);
    void set_volume(const double & volume);
    void beep(const vctDoubleVec & values); // duration, frequency, volume
    void string_to_speech(const std::string & text);
    bool mHasIO;
    void ClutchEventHandler(const prmEventButton & button);
    void CameraEventHandler(const prmEventButton & button);
    void OperatorPresentEventHandler(const prmEventButton & button);

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

    mtsInterfaceProvided * mInterface;
    struct {
        mtsFunctionWrite ArmCurrentState;
        mtsFunctionWrite scale;
        mtsFunctionWrite teleop_psm_selected;
        mtsFunctionWrite teleop_psm_unselected;
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

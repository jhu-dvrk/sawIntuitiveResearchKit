/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitConsole_h
#define _mtsIntuitiveResearchKitConsole_h

#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmKeyValue.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
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
                      ARM_MTM, ARM_PSM, ARM_ECM, ARM_SUJ,
                      ARM_MTM_GENERIC, ARM_PSM_GENERIC, ARM_ECM_GENERIC,
                      ARM_MTM_DERIVED, ARM_PSM_DERIVED, ARM_ECM_DERIVED,
                      ARM_PSM_SOCKET} ArmType;

        typedef enum {SIMULATION_NONE,
                      SIMULATION_KINEMATIC,
                      SIMULATION_DYNAMIC} SimulationType;

        friend class mtsIntuitiveResearchKitConsole;
        friend class mtsIntuitiveResearchKitConsoleQt;
        friend class dvrk::console;

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
          SetBaseFrame on the arm. */
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
        mtsIntuitiveResearchKitConsole * mConsole = nullptr;
        std::string mName;
        ArmType mType;
        bool mIsNativeOrDerived;
        std::string mSerial;
        SimulationType mSimulation;

        // low level
        std::string mIOComponentName;
        std::string mIOConfigurationFile;
        // PID
        std::string mPIDComponentName;
        std::string mPIDConfigurationFile;
        // arm
        std::string mComponentName;
        std::string mInterfaceName;
        std::string mArmConfigurationFile;
        double mArmPeriod;
        // socket
        std::string mIp;
        int mPort;
        bool mSocketServer;
        std::string mSocketComponentName;
        // generic arm
        bool mIsGeneric;
        std::string mSharedLibrary;
        std::string mClassName;
        std::string mConstructorArgJSON;
        bool mSkipROSBridge;

        // base frame
        // (name and frame) OR (component and interface)
        prmPositionCartesianSet mBaseFrame;
        std::string mBaseFrameComponentName;
        std::string mBaseFrameInterfaceName;

        mtsFunctionWrite state_command;
        mtsFunctionVoid Freeze;
        mtsInterfaceRequired * IOInterfaceRequired;
        mtsInterfaceRequired * IODallasInterfaceRequired;
        mtsInterfaceRequired * PIDInterfaceRequired;
        mtsInterfaceRequired * ArmInterfaceRequired;

        // this is used only by PSMs and ECM
        mtsInterfaceRequired * SUJInterfaceRequiredFromIO;
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
    };

    class CISST_EXPORT TeleopECM {
    public:
        typedef enum {TELEOP_ECM, TELEOP_ECM_DERIVED, TELEOP_ECM_GENERIC} TeleopECMType;
        friend class mtsIntuitiveResearchKitConsole;
        friend class mtsIntuitiveResearchKitConsoleQt;
        friend class dvrk::console;

        TeleopECM(const std::string & name,
                  const std::string & masterLeftComponentName,
                  const std::string & masterLeftInterfaceName,
                  const std::string & masterRightComponentName,
                  const std::string & masterRightInterfaceName,
                  const std::string & slaveComponentName,
                  const std::string & slaveInterfaceName,
                  const std::string & consoleName);

        /*! Create and configure the robot arm. */
        void ConfigureTeleop(const TeleopECMType type,
                             const double & periodInSeconds,
                             const Json::Value & jsonConfig);

        /*! Connect all interfaces specific to this teleop. */
        bool Connect(void);

        /*! Accessors */
        const std::string & Name(void) const;

    protected:
        std::string mName;
        TeleopECMType mType;
        std::string mMTMLComponentName;
        std::string mMTMLInterfaceName;
        std::string mMTMRComponentName;
        std::string mMTMRInterfaceName;
        std::string mECMComponentName;
        std::string mECMInterfaceName;
        std::string mConsoleName;
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
                  const std::string & masterComponentName,
                  const std::string & masterInterfaceName,
                  const std::string & namePSM,
                  const std::string & slaveComponentName,
                  const std::string & slaveInterfaceName,
                  const std::string & consoleName,
                  const std::string & baseFrameComponentName,
                  const std::string & baseFrameInterfaceName);

        /*! Create and configure the robot arm. */
        void ConfigureTeleop(const TeleopPSMType type,
                             const double & periodInSeconds,
                             const Json::Value & jsonConfig);

        /*! Connect all interfaces specific to this teleop. */
        bool Connect(void);

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
        std::string mName;
        TeleopPSMType mType;
        std::string mMTMName;
        std::string mMTMComponentName;
        std::string mMTMInterfaceName;
        std::string mPSMName;
        std::string mPSMComponentName;
        std::string mPSMInterfaceName;
        std::string mConsoleName;
        std::string mBaseFrameComponentName;
        std::string mBaseFrameInterfaceName;
        mtsFunctionWrite state_command;
        mtsFunctionWrite SetScale;
        mtsInterfaceRequired * InterfaceRequired;
    };

    mtsIntuitiveResearchKitConsole(const std::string & componentName);
    inline virtual ~mtsIntuitiveResearchKitConsole() {}

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

    // to be deprecated
    bool AddTeleOperation(const std::string & name,
                          const std::string & masterName,
                          const std::string & slaveName);

    void AddFootpedalInterfaces(void);
    void ConnectFootpedalInterfaces(void);

    bool Connect(void);

 protected:
    bool mConfigured;
    double mTimeOfLastErrorBeep;
    bool mTeleopEnabled;
    bool mTeleopPSMRunning;
    bool mTeleopPSMAligning;
    bool mTeleopECMRunning;
    bool mTeleopEnabledBeforeCamera;

    typedef std::map<std::string, Arm *> ArmList;
    ArmList mArms;

    /*! Pointer to mtsTextToSpeech component */
    mtsTextToSpeech * mTextToSpeech;

    /*! List to manage multiple PSM teleoperations */
    typedef std::map<std::string, TeleopPSM *> TeleopPSMList;
    TeleopPSMList mTeleopsPSM;

    /*! List to manage the teleopPSM components for each MTM */
    typedef std::multimap<std::string, TeleopPSM *> TeleopPSMByMTMList;
    typedef TeleopPSMByMTMList::iterator TeleopPSMByMTMIterator;
    typedef TeleopPSMByMTMList::const_iterator TeleopPSMByMTMConstIterator;
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
                          const std::string & ioComponentName,
                          const cmnPath & configPath);
    bool AddArmInterfaces(Arm * arm);

    // these two methods have exact same implementation.it would be
    // nice to have a base class, or template this
    bool AddTeleopECMInterfaces(TeleopECM * teleop);
    bool AddTeleopPSMInterfaces(TeleopPSM * teleop);

    bool ConfigureECMTeleopJSON(const Json::Value & jsonTeleop);
    bool ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop);

    void PowerOff(void);
    void PowerOn(void);
    void Home(void);
    void TeleopEnable(const bool & enable);
    void CycleTeleopPSMByMTM(const std::string & mtmName);
    void SelectTeleopPSM(const prmKeyValue & mtmPsm);
    bool GetPSMSelectedForMTM(const std::string & mtmName, std::string & psmName) const;
    bool GetMTMSelectedForPSM(const std::string & psmName, std::string & mtmName) const;
    void EventSelectedTeleopPSMs(void) const;
    void UpdateTeleopState(void);
    void SetScale(const double & scale);
    void SetVolume(const double & volume);
    void StringToSpeech(const std::string & text);
    bool mHasIO;
    void ClutchEventHandler(const prmEventButton & button);
    void CameraEventHandler(const prmEventButton & button);
    void OperatorPresentEventHandler(const prmEventButton & button);

    struct {
        mtsFunctionWrite Beep;
        mtsFunctionWrite StringToSpeech;
    } mAudio;
    double mAudioVolume;
    bool mChatty;

    struct {
        mtsFunctionWrite Clutch;
        mtsFunctionWrite Camera;
        mtsFunctionWrite OperatorPresent;
    } ConsoleEvents;
    bool mOperatorPresent;
    bool mCameraPressed;

    std::string mIOComponentName; // for actuator IOs

    // components used for events (digital inputs)
    typedef std::pair<std::string, std::string> InterfaceComponentType;
    typedef std::map<std::string, InterfaceComponentType> DInputSourceType;
    DInputSourceType mDInputSources;

    mtsInterfaceProvided * mInterface;
    struct {
        mtsFunctionWrite ArmCurrentState;
        mtsFunctionWrite Scale;
        mtsFunctionWrite TeleopPSMSelected;
        mtsFunctionWrite TeleopPSMUnselected;
    } ConfigurationEvents;

    void ErrorEventHandler(const mtsMessage & message);
    void WarningEventHandler(const mtsMessage & message);
    void StatusEventHandler(const mtsMessage & message);

    void SetArmCurrentState(const std::string & armName,
                            const prmOperatingState & currentState);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitConsole);

#endif // _mtsIntuitiveResearchKitConsole_h

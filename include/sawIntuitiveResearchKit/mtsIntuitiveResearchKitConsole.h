/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitConsole_h
#define _mtsIntuitiveResearchKitConsole_h

#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

namespace dvrk {
    class console;
}

class mtsIntuitiveResearchKitConsole: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    friend class mtsIntuitiveResearchKitConsoleQt;
    friend class dvrk::console;

    class Arm {
    public:
        typedef enum {ARM_UNDEFINED, ARM_MTM, ARM_PSM, ARM_ECM, ARM_SUJ, ARM_MTM_GENERIC, ARM_PSM_GENERIC, ARM_MTM_DERIVED, ARM_PSM_DERIVED} ArmType;

        friend class mtsIntuitiveResearchKitConsole;
        friend class mtsIntuitiveResearchKitConsoleQt;
        friend class dvrk::console;

        Arm(const std::string & name,
            const std::string & ioComponentName);

        /*! Create a new PID component and connect it to the proper RobotIO
          interface.  If the period in seconds is zero, the PID will be tied to
          IO using the ExecIn/ExecOut interfaces. */
        void ConfigurePID(const std::string & configFile,
                          const double & periodInSeconds = 0.0 * cmn_ms);

        /*! Create and configure the robot arm. */
        void ConfigureArm(const ArmType armType,
                          const std::string & configFile,
                          const double & periodInSeconds = 0.5 * cmn_ms,
                          mtsComponent * existingArm = 0);

        /*! Connect all interfaces specific to this arm. */
        bool Connect(void);

        /*! Accessors */
        const std::string & Name(void) const;
        const std::string & IOComponentName(void) const;
        const std::string & PIDComponentName(void) const;

    protected:
        std::string mName;
        ArmType mType;

        // low level
        std::string mIOComponentName;
        std::string mIOConfigurationFile;
        // PID
        std::string mPIDComponentName;
        std::string mPIDConfigurationFile;
        // arm
        std::string mArmConfigurationFile;
        // base frame
        std::string mBaseFrameComponentName;
        std::string mBaseFrameInterfaceName;

        mtsFunctionWrite SetRobotControlState;
        mtsInterfaceRequired * IOInterfaceRequired;
        mtsInterfaceRequired * PIDInterfaceRequired;
        mtsInterfaceRequired * ArmInterfaceRequired;

        // this is used only by PSMs and ECM
        mtsInterfaceRequired * SUJInterfaceRequiredFromIO;
        mtsInterfaceRequired * SUJInterfaceRequiredToSUJ;
        mtsFunctionWrite SUJClutch;

        void SUJClutchEventHandlerFromIO(const prmEventButton & button) {
            if (button.Type() == prmEventButton::PRESSED) {
                SUJClutch(true);
            } else {
                SUJClutch(false);
            }
        }
    };

    class TeleopPSM {
    public:
        typedef enum {TELEOP_PSM, TELEOP_PSM_GENERIC} TeleopPSMType;
        friend class mtsIntuitiveResearchKitConsole;
        friend class mtsIntuitiveResearchKitConsoleQt;
        friend class dvrk::console;

        TeleopPSM(const std::string & name,
                  const std::string & masterName,
                  const std::string & slaveName,
                  const std::string & consoleName);

        /*! Create and configure the robot arm. */
        void ConfigureTeleop(const TeleopPSMType type,
                             const vctMatRot3 & orientation,
                             const double & periodInSeconds = 2.0 * cmn_ms);

        /*! Connect all interfaces specific to this teleop. */
        bool Connect(void);

        /*! Accessors */
        const std::string & Name(void) const;

    protected:
        std::string mName;
        TeleopPSMType mType;
        std::string mMasterName;
        std::string mSlaveName;
        std::string mConsoleName;
        mtsFunctionWrite Enable;
        mtsFunctionWrite ManipClutch;
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

    bool AddFootpedalInterfaces(void);
    bool ConnectFootpedalInterfaces(void);

    bool Connect(void);

protected:
    bool mConfigured;

    typedef std::map<std::string, Arm *> ArmList;
    ArmList mArms;

    typedef std::map<std::string, TeleopPSM *> TeleopList;
    TeleopList mTeleops;

    /*! Utility function to test if a file exists and log the results */
    bool FileExists(const std::string & description, const std::string & filename) const;

    /*! Find all arm data from JSON configuration. */
    bool ConfigureArmJSON(const Json::Value & jsonArm,
                          const std::string & ioComponentName);
    bool AddArmInterfaces(Arm * arm);

    bool ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop);
    bool AddTeleopInterfaces(TeleopPSM * teleop);


    void SetRobotsControlState(const std::string & newState);
    void TeleopEnable(const bool & enable);

    bool mHasFootpedals;
    void ClutchEventHandler(const prmEventButton & button);
    void CameraEventHandler(const prmEventButton & button);
    void OperatorPresentEventHandler(const prmEventButton & button);
    struct {
        mtsFunctionWrite Clutch;
        mtsFunctionWrite Camera;
        mtsFunctionWrite OperatorPresent;
    } ConsoleEvents;
    std::string mIOComponentName;
    std::string mOperatorPresentComponent;
    std::string mOperatorPresentInterface;

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
    } MessageEvents;

    void ErrorEventHandler(const std::string & message);
    void WarningEventHandler(const std::string & message);
    void StatusEventHandler(const std::string & message);

    void ECMManipClutchEventHandler(const prmEventButton & button);

    // Getting position from ECM and ECM SUJ to create base frame event for all other SUJs
    mtsInterfaceRequired * mSUJECMInterfaceRequired;
    mtsInterfaceProvided * mECMBaseFrameInterfaceProvided;
    mtsFunctionRead mGetPositionCartesianLocalFromECM;
    mtsFunctionWrite mECMBaseFrameEvent;
    void SUJECMBaseFrameHandler(const prmPositionCartesianGet & baseFrameParam);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitConsole);

#endif // _mtsIntuitiveResearchKitConsole_h

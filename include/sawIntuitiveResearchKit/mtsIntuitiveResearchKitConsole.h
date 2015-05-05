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

class mtsIntuitiveResearchKitConsole: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:

    class Arm {
    public:
        typedef enum {ARM_MTM, ARM_PSM, ARM_ECM, ARM_GENERIC_MTM, ARM_GENERIC_PSM} ArmType;

        friend class mtsIntuitiveResearchKitConsole;

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
                          const double & periodInSeconds = 0.0 * cmn_ms,
                          mtsComponent * existingArm = 0);

        /*! Accessors */
        const std::string & Name(void) const;
        const std::string & IOComponentName(void) const;
        const std::string & PIDComponentName(void) const;

    protected:
        std::string mName;
        // low level
        std::string mIOComponentName;
        std::string mIOConfigurationFile;
        // PID
        std::string mPIDComponentName;
        std::string mPIDConfigurationFile;
        // arm
        std::string mArmConfigurationFile;

        mtsFunctionWrite SetRobotControlState;
        mtsInterfaceRequired * IOInterfaceRequired;
        mtsInterfaceRequired * PIDInterfaceRequired;
        mtsInterfaceRequired * ArmInterfaceRequired;
    };

    class TeleOp {
    public:
        friend class mtsIntuitiveResearchKitConsole;

        TeleOp(const std::string & name);

        /*! Accessors */
        const std::string & Name(void) const;

    protected:
        std::string mName;
        mtsFunctionWrite Enable;
        mtsInterfaceRequired * InterfaceRequired;
    };

    mtsIntuitiveResearchKitConsole(const std::string & componentName);
    inline virtual ~mtsIntuitiveResearchKitConsole() {}

    /*! Configure console using JSON file.  To test is the configuration succeeded, used method Configured().
      */
    void Configure(const std::string & filename);
    const bool & Configured(void) const;
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    bool AddArm(Arm * newMTM);
    bool AddArm(mtsComponent * genericArm, const Arm::ArmType armType);
    bool AddTeleOperation(const std::string & name);

protected:

    bool mConfigured;

    typedef std::list<Arm *> ArmList;
    ArmList mArms;

    typedef std::list<TeleOp *> TeleOpList;
    TeleOpList mTeleOps;

    bool SetupAndConnectInterfaces(Arm * arm);

    void SetRobotsControlState(const std::string & newState);
    void TeleopEnable(const bool & enable);

    void ClutchEventHandler(const prmEventButton & button);
    void CameraEventHandler(const prmEventButton & button);
    void OperatorPresentEventHandler(const prmEventButton & button);
    struct {
        mtsFunctionWrite Clutch;
        mtsFunctionWrite Camera;
        mtsFunctionWrite OperatorPresent;
    } ConsoleEvents;

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
    } MessageEvents;

    void ErrorEventHandler(const std::string & message);
    void WarningEventHandler(const std::string & message);
    void StatusEventHandler(const std::string & message);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitConsole);

#endif // _mtsIntuitiveResearchKitConsole_h

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

#include <sawIntuitiveResearchKit/system_configuration.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {
    class IO_proxy;
    class arm_proxy;
    class teleop_PSM_proxy;
    class teleop_ECM_proxy;
}

namespace dvrk_ros {
    class console;
}

class mtsRobotIO1394;
class mtsTextToSpeech;
class mtsDaVinciEndoscopeFocus;
class mtsIntuitiveResearchKitArm;

class CISST_EXPORT mtsIntuitiveResearchKitConsole: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    friend class mtsIntuitiveResearchKitConsoleQt;
    friend class dvrk::arm_proxy;
    friend class dvrk_ros::console;
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

    void AddFootpedalInterfaces(void);

    bool Connect(void);

    std::string find_file(const std::string & filename) const;

 protected:
    dvrk::system_configuration m_config;
    bool m_configured;
    cmnPath m_config_path;
    mtsDelayedConnections m_connections;

    double mTimeOfLastErrorBeep;

    /*! Pointer to mtsTextToSpeech component */
    std::shared_ptr<mtsTextToSpeech> m_text_to_speech;

    typedef std::map<std::string, std::shared_ptr<dvrk::IO_proxy>> IO_proxies;
    IO_proxies m_IO_proxies;

    typedef std::map<std::string, std::shared_ptr<dvrk::arm_proxy>> arm_proxies;
    arm_proxies m_arm_proxies;

    /*! List to manage multiple ECM teleoperations */

    /*! Head sensor */
    mtsComponent * mHeadSensor = nullptr;

    /*! daVinci Endoscope Focus */
    mtsDaVinciEndoscopeFocus * mDaVinciEndoscopeFocus = nullptr;

    bool add_IO_interfaces(std::shared_ptr<dvrk::IO_proxy> IO_proxy);
    bool add_arm_interfaces(std::shared_ptr<dvrk::arm_proxy> arm_proxy);

    // these two methods have exact same implementation.it would be
    // nice to have a base class, or template this
    bool add_teleop_PSM_interfaces(std::shared_ptr<dvrk::teleop_PSM_proxy> teleop_proxy);
    bool add_teleop_ECM_interfaces(std::shared_ptr<dvrk::teleop_ECM_proxy> teleop_proxy);

    void power_off(void);
    void power_on(void);
    void home(void);
    void DisableFaultyArms(void);
    void set_scale(const double & scale);
    void set_volume(const double & volume);
    void beep(const vctDoubleVec & values); // duration, frequency, volume
    void string_to_speech(const std::string & text);
    std::shared_ptr<dvrk::arm_proxy> m_SUJ = nullptr;
    // void ClutchEventHandler(const prmEventButton & button);
    // void CameraEventHandler(const prmEventButton & button);
    // void OperatorPresentEventHandler(const prmEventButton & button);

    void ConnectInternal(bool &ret) const;

    bool m_calibration_mode = false;

    struct {
        mtsFunctionWrite beep;
        mtsFunctionWrite string_to_speech;
        mtsFunctionWrite volume; // event
    } audio;
    double m_audio_volume;

    // struct {
    //     mtsFunctionWrite clutch;
    //     mtsFunctionWrite camera;
    //     mtsFunctionWrite operator_present;
    //     mtsFunctionWrite teleop_enabled;
    // } console_events;

    // components used for events (digital inputs)
    typedef std::pair<std::string, std::string> InterfaceComponentType;
    typedef std::map<std::string, InterfaceComponentType> DInputSourceType;
    DInputSourceType mDInputSources;

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

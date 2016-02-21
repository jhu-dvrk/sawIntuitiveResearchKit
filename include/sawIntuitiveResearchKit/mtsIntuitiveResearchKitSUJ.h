/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Youri Tan
  Created on: 2014-11-07

  (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitSUJ_h
#define _mtsIntuitiveResearchKitSUJ_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// forward declaration
class mtsIntuitiveResearchKitSUJArmData;

class CISST_EXPORT mtsIntuitiveResearchKitSUJ: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    static const size_t NumberOfJoints = 4;
    static const size_t NumberOfBrakes = 3;

    mtsIntuitiveResearchKitSUJ(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitSUJ(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitSUJ() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    void Init(void);

    /*! Get data from the PID level based on current state. */
    void GetRobotData(void);

    /*! Logic used to read the potentiometer values and updated the
      appropriate joint values based on the mux state. */
    void GetAndConvertPotentiometerValues(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState);

    /*! Homing procedure, will check the homing state and call the required method. */
    void RunHoming(void);

    /*! Homing procedure, power the robot and initial current and encoder calibration. */
    void RunHomingPower(void);

    /*! Normal operations. */
    void RunReady(void);

    void SetRobotControlState(const std::string & state);

    /*! Convert enum to string using function provided by cisstDataGenerator. */
    void GetRobotControlState(std::string & state) const;

    /*! Set velocity for motorized PSM lift. normalized between -1.0 and 1.0. */
    void SetLiftVelocity(const double & velocity);

    /*! Set base coordinate frame, this will be added to the kinematics of all SUJs except ECM. */
    void SetBaseFrame(const prmPositionCartesianGet & newBaseFrame);

    /*! Event handler for PID errors. */
    void ErrorEventHandler(const std::string & message);

    /*! Motor down button. */
    void MotorDownEventHandler(const prmEventButton & button);

    /*! Motor up button. */
    void MotorUpEventHandler(const prmEventButton & button);

    // Required interface
    struct {
        //! Enable Robot Power
        mtsFunctionVoid EnablePower;
        mtsFunctionVoid DisablePower;
        mtsFunctionRead GetEncoderChannelA;
        mtsFunctionRead GetActuatorAmpStatus;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionRead GetAnalogInputVolts;
    } RobotIO;

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
        mtsFunctionWrite RobotState;
    } MessageEvents;

    // Functions to control MUX
    struct {
        mtsFunctionRead GetValue;
        mtsFunctionWrite SetValue;
    } MuxReset;

    struct {
        mtsFunctionRead GetValue;
        mtsFunctionWrite SetValue;
    } MuxIncrement;

    double mMuxTimer;
    vctBoolVec mMuxState;
    size_t mMuxIndex, mMuxIndexExpected;

    // Functions to control motor on SUJ3
    struct {
        mtsFunctionWrite EnablePWM;
        mtsFunctionWrite SetPWMDutyCycle;
    } PWM;

    mtsIntuitiveResearchKitArmTypes::RobotStateType mRobotState;

    // Home Action
    double mHomingTimer;
    bool mHomingPowerRequested;

    // Clutch / brake timer
    double mPreviousTic;
    vctDoubleVec mBrakeCurrents;

    vctDynamicVector<vctDoubleVec> mVoltageSamples;
    const size_t mVoltageSamplesNumber;
    size_t mVoltageSamplesCounter;
    vctDoubleVec mVoltages;
    vctFixedSizeVector<mtsIntuitiveResearchKitSUJArmData *, 4> Arms;

    void DispatchError(const std::string & message);
    void DispatchStatus(const std::string & message);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitSUJ);

#endif // _mtsIntuitiveResearchKitSUJ_h

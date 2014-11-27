/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitECM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitECM::mtsIntuitiveResearchKitECM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitECM::mtsIntuitiveResearchKitECM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init();
}

void mtsIntuitiveResearchKitECM::Init(void)
{
    IsCartesianGoalSet = false;
    Counter = 0;

    SetState(mtsIntuitiveResearchKitECMTypes::ECM_UNINITIALIZED);

    // initialize trajectory data
    JointGet.SetSize(NumberOfJoints);
    JointSet.SetSize(NumberOfJoints);
    JointSetParam.Goal().SetSize(NumberOfJoints);
    JointTrajectory.Velocity.SetSize(NumberOfJoints);
    JointTrajectory.Velocity.Assign(30.0 * cmnPI_180, // degrees per second
                                    30.0 * cmnPI_180,
                                     0.05,            // m per second
                                    20.0 * cmnPI_180);
    JointTrajectory.Acceleration.SetSize(NumberOfJoints);
    JointTrajectory.Acceleration.Assign(30.0 * cmnPI_180,
                                        30.0 * cmnPI_180,
                                         0.05,
                                        30.0 * cmnPI_180);
    JointTrajectory.Start.SetSize(NumberOfJoints);
    JointTrajectory.Goal.SetSize(NumberOfJoints);
    JointTrajectory.GoalError.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI / 180.0); // hard coded to 3 degrees

    this->StateTable.AddData(CartesianGetParam, "CartesianPosition");
    this->StateTable.AddData(CartesianGetDesiredParam, "CartesianDesired");
    this->StateTable.AddData(JointGetParam, "JointPosition");

    // setup CISST Interface
    mtsInterfaceRequired * interfaceRequired;
    interfaceRequired = AddInterfaceRequired("PID");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("Enable", PID.Enable);
        interfaceRequired->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        interfaceRequired->AddFunction("GetPositionJointDesired", PID.GetPositionJointDesired);
        interfaceRequired->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        interfaceRequired->AddFunction("SetCheckJointLimit", PID.SetCheckJointLimit);
        interfaceRequired->AddFunction("EnableTrackingError", PID.EnableTrackingError);
        interfaceRequired->AddFunction("SetTrackingErrorTolerances", PID.SetTrackingErrorTolerance);
        interfaceRequired->AddEventHandlerVoid(&mtsIntuitiveResearchKitECM::EventHandlerTrackingError, this, "TrackingError");
    }

    // Robot IO
    interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("EnablePower", RobotIO.EnablePower);
        interfaceRequired->AddFunction("DisablePower", RobotIO.DisablePower);
        interfaceRequired->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        interfaceRequired->AddFunction("GetBrakeAmpStatus", RobotIO.GetBrakeAmpStatus);
        interfaceRequired->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
        interfaceRequired->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        interfaceRequired->AddFunction("UsePotsForSafetyCheck", RobotIO.UsePotsForSafetyCheck);
        interfaceRequired->AddFunction("SetPotsToEncodersTolerance", RobotIO.SetPotsToEncodersTolerance);
        interfaceRequired->AddFunction("BrakeRelease", RobotIO.BrakeRelease);
        interfaceRequired->AddFunction("BrakeEngage", RobotIO.BrakeEngage);
    }

    // ManipClutch: digital input button event from ECM
    interfaceRequired = AddInterfaceRequired("ManipClutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECM::EventHandlerManipClutch, this, "Button");
    }

    // SUJClutch: digital input button event from ECM
    interfaceRequired = AddInterfaceRequired("SUJClutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECM::EventHandlerSUJClutch, this, "Button");
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        // Cartesian
        interfaceProvided->AddCommandReadState(this->StateTable, JointGetParam, "GetPositionJoint");
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianGetParam, "GetPositionCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianGetDesiredParam, "GetPositionCartesianDesired");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitECM::SetPositionCartesian, this, "SetPositionCartesian");
        // Robot State
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitECM::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddCommandRead(&mtsIntuitiveResearchKitECM::GetRobotControlState,
                                          this, "GetRobotControlState", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotStatusMsg, "RobotStatusMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotErrorMsg, "RobotErrorMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.ManipClutch, "ManipClutchBtn", prmEventButton());
        interfaceProvided->AddEventWrite(EventTriggers.SUJClutch, "SUJClutchBtn", prmEventButton());
        // Stats
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }
}

void mtsIntuitiveResearchKitECM::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
    }
}

void mtsIntuitiveResearchKitECM::Startup(void)
{
    this->SetState(mtsIntuitiveResearchKitECMTypes::ECM_UNINITIALIZED);
}

void mtsIntuitiveResearchKitECM::Run(void)
{
    Counter++;

    ProcessQueuedEvents();
    GetRobotData();

    switch (RobotState) {
    case mtsIntuitiveResearchKitECMTypes::ECM_UNINITIALIZED:
        break;
    case mtsIntuitiveResearchKitECMTypes::ECM_HOMING_POWERING:
        RunHomingPower();
        break;
    case mtsIntuitiveResearchKitECMTypes::ECM_HOMING_CALIBRATING_ARM:
        RunHomingCalibrateArm();
        break;
    case mtsIntuitiveResearchKitECMTypes::ECM_READY:
    case mtsIntuitiveResearchKitECMTypes::ECM_POSITION_CARTESIAN:
        RunPositionCartesian();
        break;
    case mtsIntuitiveResearchKitECMTypes::ECM_MANUAL:
        break;
    default:
        break;
    }

    RunEvent();
    ProcessQueuedCommands();
}

void mtsIntuitiveResearchKitECM::Cleanup(void)
{
    RobotIO.BrakeEngage();
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitECM::GetRobotData(void)
{
    // we can start reporting some joint values after the robot is powered
    if (this->RobotState > mtsIntuitiveResearchKitECMTypes::ECM_HOMING_POWERING) {
        mtsExecutionResult executionResult;
        executionResult = PID.GetPositionJoint(JointGetParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
        }

        // angles are radians but cisst uses mm.   robManipulator uses SI, so we need meters
        JointGetParam.Position().Element(2) /= 1000.0;  // convert from mm to m

        // assign to a more convenient vctDoubleVec
        JointGet.Assign(JointGetParam.Position(), NumberOfJoints);

        // desired joints
        executionResult = PID.GetPositionJointDesired(JointGetDesired);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPositionDesired failed \""
                                    << executionResult << "\"" << std::endl;
        }
        // angles are radians but cisst uses mm.   robManipulator uses SI, so we need meters
        JointGetDesired.Element(2) /= 1000.0;  // convert from mm to m

        // when the robot is ready, we can compute cartesian position
        if (this->RobotState >= mtsIntuitiveResearchKitECMTypes::ECM_READY) {
            // update cartesian position
            CartesianGet = Manipulator.ForwardKinematics(JointGet);
            CartesianGet.Rotation().NormalizedSelf();
            CartesianGetParam.SetValid(true);
            // update cartesian position desired based on joint desired
            CartesianGetDesired = Manipulator.ForwardKinematics(JointGetDesired);
            CartesianGetDesired.Rotation().NormalizedSelf();
            CartesianGetDesiredParam.SetValid(true);
        } else {
            // update cartesian position
            CartesianGet.Assign(vctFrm4x4::Identity());
            CartesianGetParam.SetValid(false);
            // update cartesian position desired
            CartesianGetDesired.Assign(vctFrm4x4::Identity());
            CartesianGetDesiredParam.SetValid(false);
        }
        CartesianGetParam.Position().From(CartesianGet);
        CartesianGetDesiredParam.Position().From(CartesianGetDesired);
    } else {
        // set joint to zeros
        JointGet.Zeros();
        JointGetParam.Position().Zeros();
        JointGetParam.SetValid(false);
    }
}

void mtsIntuitiveResearchKitECM::SetState(const mtsIntuitiveResearchKitECMTypes::RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state "
                            << mtsIntuitiveResearchKitECMTypes::RobotStateTypeToString(newState) << std::endl;

    switch (newState) {

    case mtsIntuitiveResearchKitECMTypes::ECM_UNINITIALIZED:
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " not initialized");
        break;

    case mtsIntuitiveResearchKitECMTypes::ECM_HOMING_POWERING:
        HomingTimer = 0.0;
        HomingPowerRequested = false;
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " powering");
        break;

    case mtsIntuitiveResearchKitECMTypes::ECM_HOMING_CALIBRATING_ARM:
        HomingCalibrateArmStarted = false;
        RobotState = newState;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " calibrating arm");
        break;

    case mtsIntuitiveResearchKitECMTypes::ECM_READY:
        // when returning from manual mode, need to re-enable PID
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " ready");
        break;

    case mtsIntuitiveResearchKitECMTypes::ECM_POSITION_CARTESIAN:
        if (this->RobotState < mtsIntuitiveResearchKitECMTypes::ECM_READY) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not ready");
            return;
        }
        // check that the tool is inserted deep enough
        if (JointGet.Element(2) < 80.0 / 1000.0) {
            EventTriggers.RobotErrorMsg(this->GetName() + " can't start cartesian mode, make sure the tool is inserted past the cannula");
            break;
        }
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " position cartesian");
        break;

    case mtsIntuitiveResearchKitECMTypes::ECM_CONSTRAINT_CONTROLLER_CARTESIAN:
        if (this->RobotState < mtsIntuitiveResearchKitECMTypes::ECM_READY) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not ready");
            return;
        }
        // check that the tool is inserted deep enough
        if (JointGet.Element(2) < 80.0 / 1000.0) {
            EventTriggers.RobotErrorMsg(this->GetName() + " can't start constraint controller cartesian mode, make sure the tool is inserted past the cannula");
            break;
        }
        RobotState = newState;
        IsCartesianGoalSet = false;
        EventTriggers.RobotStatusMsg(this->GetName() + " constraint controller cartesian");
        break;

    case mtsIntuitiveResearchKitECMTypes::ECM_MANUAL:
        if (this->RobotState < mtsIntuitiveResearchKitECMTypes::ECM_READY) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not ready yet");
            return;
        }
        // disable PID to allow manual move
        PID.Enable(false);
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " in manual mode");
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitECM::RunHomingPower(void)
{
    const double timeToPower = 3.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!HomingPowerRequested) {
        // in case we still have power but brakes are not engaged
        RobotIO.BrakeEngage();
        // bias encoders based on pots
        RobotIO.BiasEncoder();
        { // use pots for redundancy
            vctDoubleVec potsToEncodersTolerance(this->NumberOfJoints);
            potsToEncodersTolerance.SetAll(10.0 * cmnPI_180); // 10 degrees for rotations
            potsToEncodersTolerance.Element(2) = 20.0; // 20 mm
            RobotIO.SetPotsToEncodersTolerance(potsToEncodersTolerance);
            RobotIO.UsePotsForSafetyCheck(true);
        }
        HomingTimer = currentTime;
        // make sure the PID is not sending currents
        PID.Enable(false);
        // pre-load the boards with zero current
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfJoints, 0.0));
        // enable power and set a flags to move to next step
        RobotIO.EnablePower();
        HomingPowerRequested = true;
        EventTriggers.RobotStatusMsg(this->GetName() + " power requested");
        return;
    }

    // second, check status
    if (HomingPowerRequested
        && ((currentTime - HomingTimer) > timeToPower)) {

        // pre-load PID to make sure desired position has some reasonable values
        PID.GetPositionJoint(JointGetParam);
        // angles are radians but cisst uses mm.   robManipulator uses SI, so we need meters
        JointGetParam.Position().Element(2) /= 1000.0;  // convert from mm to m
        // assign to a more convenient vctDoubleVec
        JointGet.Assign(JointGetParam.Position(), NumberOfJoints);
        SetPositionJointLocal(JointGet);

        // check power status
        vctBoolVec actuatorAmplifiersStatus(NumberOfJoints);
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        vctBoolVec brakeAmplifiersStatus(NumberOfBrakes);
        RobotIO.GetBrakeAmpStatus(brakeAmplifiersStatus);
        if (actuatorAmplifiersStatus.All() && brakeAmplifiersStatus.All()) {
            EventTriggers.RobotStatusMsg(this->GetName() + " power on");
            RobotIO.BrakeRelease();
            this->SetState(mtsIntuitiveResearchKitECMTypes::ECM_HOMING_CALIBRATING_ARM);
        } else {
            EventTriggers.RobotErrorMsg(this->GetName() + " failed to enable power.");
            this->SetState(mtsIntuitiveResearchKitECMTypes::ECM_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitECM::RunHomingCalibrateArm(void)
{
    static const double extraTime = 5.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    // trigger motion
    if (!HomingCalibrateArmStarted) {
        // disable joint limits
        PID.SetCheckJointLimit(false);
        // enable PID and start from current position
        JointSet.ForceAssign(JointGet);
        SetPositionJointLocal(JointSet);
        // configure PID to fail in case of tracking error
        vctDoubleVec tolerances(NumberOfJoints);
        tolerances.SetAll(7.0 * cmnPI_180); // 5 degrees on angles
        tolerances.Element(2) = 10.0; // 5 mm
        PID.SetTrackingErrorTolerance(tolerances);
        PID.EnableTrackingError(true);
        // finally enable PID
        PID.Enable(true);

        // compute joint goal position
        JointTrajectory.Goal.SetSize(NumberOfJoints);
        JointTrajectory.Goal.ForceAssign(JointGet);
        JointTrajectory.Goal.SetAll(0.0);
        JointTrajectory.LSPB.Set(JointGet, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
        // set flag to indicate that homing has started
        HomingCalibrateArmStarted = true;
    }

    // compute a new set point based on time
    if (currentTime <= HomingTimer) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    } else {
        // request final position in case trajectory rounding prevent us to get there
        SetPositionJointLocal(JointTrajectory.Goal);

        // check position
        JointTrajectory.GoalError.DifferenceOf(JointTrajectory.Goal, JointGet);
        JointTrajectory.GoalError.AbsSelf();
        bool isHomed = !JointTrajectory.GoalError.ElementwiseGreaterOrEqual(JointTrajectory.GoalTolerance).Any();
        if (isHomed) {
            PID.SetCheckJointLimit(true);
            EventTriggers.RobotStatusMsg(this->GetName() + " arm ready");
            this->SetState(mtsIntuitiveResearchKitECMTypes::ECM_READY);
        } else {
            // time out
            if (currentTime > HomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingCalibrateArm: unable to reach home position, error in degrees is "
                                           << JointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                EventTriggers.RobotErrorMsg(this->GetName() + " unable to reach home position during calibration on pots.");
                PID.Enable(false);
                PID.SetCheckJointLimit(true);
                this->SetState(mtsIntuitiveResearchKitECMTypes::ECM_UNINITIALIZED);
            }
        }
    }
}

void mtsIntuitiveResearchKitECM::RunPositionCartesian(void)
{
    //! \todo: should prevent user to go to close to RCM!

    if (IsCartesianGoalSet == true) {
        vctDoubleVec jointSet(JointGet);

        // compute desired slave position
        CartesianPositionFrm.From(CartesianSetParam.Goal());
        Manipulator.InverseKinematics(jointSet, CartesianPositionFrm);
        jointSet.resize(4);

        // find closest solution mod 2 pi
        const double difference = JointGet[3] - jointSet[3];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[3] = jointSet[3] + differenceInTurns * 2.0 * cmnPI;

        // finally send new joint values
        SetPositionJointLocal(jointSet);

        // reset flag
        IsCartesianGoalSet = false;
    }
}

void mtsIntuitiveResearchKitECM::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints);
    JointSetParam.Goal().Element(2) *= 1000.0; // convert from meters to mm
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitECM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if ((RobotState == mtsIntuitiveResearchKitECMTypes::ECM_POSITION_CARTESIAN)
        || (RobotState == mtsIntuitiveResearchKitECMTypes::ECM_CONSTRAINT_CONTROLLER_CARTESIAN)) {
        CartesianSetParam = newPosition;
        IsCartesianGoalSet = true;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionCartesian: ECM not ready" << std::endl;
    }
}

void mtsIntuitiveResearchKitECM::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(mtsIntuitiveResearchKitECMTypes::ECM_HOMING_POWERING);
    } else if ((state == "Cartesian position") || (state == "Teleop")) {
        SetState(mtsIntuitiveResearchKitECMTypes::ECM_POSITION_CARTESIAN);
    } else if (state == "Manual") {
        SetState(mtsIntuitiveResearchKitECMTypes::ECM_MANUAL);
    } else {
        EventTriggers.RobotErrorMsg(this->GetName() + ": unsupported state " + state);
    }
}

void mtsIntuitiveResearchKitECM::EventHandlerTrackingError(void)
{
    RobotIO.DisablePower();
    EventTriggers.RobotErrorMsg(this->GetName() + ": PID tracking error");
    SetState(mtsIntuitiveResearchKitECMTypes::ECM_UNINITIALIZED);
}

void mtsIntuitiveResearchKitECM::EventHandlerManipClutch(const prmEventButton &button)
{
    // Pass events
    EventTriggers.ManipClutch(button);

    // Start manual mode but save the previous state
    if (button.Type() == prmEventButton::PRESSED) {
        EventTriggers.ManipClutchPreviousState = this->RobotState;
        SetState(mtsIntuitiveResearchKitECMTypes::ECM_MANUAL);
    } else {
        if (RobotState == mtsIntuitiveResearchKitECMTypes::ECM_MANUAL) {
            // Enable PID
            PID.Enable(true);
            // set command joint position to joint current
            JointSet.ForceAssign(JointGet);
            SetPositionJointLocal(JointSet);
            // go back to state before clutching
            SetState(EventTriggers.ManipClutchPreviousState);
        }
    }
}

void mtsIntuitiveResearchKitECM::EventHandlerSUJClutch(const prmEventButton &button)
{
    // Pass events
    EventTriggers.SUJClutch(button);

    // Log events
    if (button.Type() == prmEventButton::PRESSED) {
        CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": EventHandlerSUJClutch: pressed" << std::endl;
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": EventHandlerSUJClutch: released" << std::endl;
    }
}


void mtsIntuitiveResearchKitECM::GetRobotControlState(std::string & state) const
{
    state = mtsIntuitiveResearchKitECMTypes::RobotStateTypeToString(this->RobotState);
}

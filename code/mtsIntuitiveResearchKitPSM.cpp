/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitPSM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init();
}

void mtsIntuitiveResearchKitPSM::Init(void)
{
    Counter = 0;

    SetState(PSM_UNINITIALIZED);
    DesiredOpenAngle = 0 * cmnPI_180;

    // initialize trajectory data
    JointCurrent.SetSize(NumberOfJoints);
    JointDesired.SetSize(NumberOfJoints);
    JointDesiredParam.Goal().SetSize(NumberOfJoints);
    JointTrajectory.Start.SetSize(NumberOfJoints);
    JointTrajectory.Velocity.SetSize(NumberOfJoints);
    JointTrajectory.Acceleration.SetSize(NumberOfJoints);
    JointTrajectory.Goal.SetSize(NumberOfJoints);
    JointTrajectory.GoalError.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI / 180.0); // hard coded to 3 degrees
    JointTrajectory.Zero.SetSize(NumberOfJoints);

    EngagingJointSet.SetSize(NumberOfJoints);

    this->StateTable.AddData(CartesianCurrentParam, "CartesianPosition");
    this->StateTable.AddData(JointCurrentParam, "JointPosition");

    // setup CISST Interface
    mtsInterfaceRequired * interfaceRequired;
    interfaceRequired = AddInterfaceRequired("PID");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("Enable", PID.Enable);
        interfaceRequired->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        interfaceRequired->AddFunction("GetPositionJointDesired", PID.GetPositionJointDesired);
        interfaceRequired->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        interfaceRequired->AddFunction("SetCheckJointLimit", PID.SetCheckJointLimit);
    }

    // Robot IO
    interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("EnablePower", RobotIO.EnablePower);
        interfaceRequired->AddFunction("DisablePower", RobotIO.DisablePower);
        interfaceRequired->AddFunction("GetAmpStatus", RobotIO.GetAmpStatus);
        interfaceRequired->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
        interfaceRequired->AddFunction("BiasCurrent", RobotIO.BiasCurrent);
        interfaceRequired->AddFunction("SetMotorCurrent", RobotIO.SetMotorCurrent);
    }

    // Event Adapter engage: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("Adapter");
    if (interfaceRequired) {
        Adapter.IsPresent = false;
        interfaceRequired->AddFunction("GetButton", Adapter.GetButton);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerAdapter, this, "Button");
    }

    // Event Tool engage: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("Tool");
    if (interfaceRequired) {
        Tool.IsPresent = false;
        interfaceRequired->AddFunction("GetButton", Tool.GetButton);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerTool, this, "Button");
    }

    // ManipClutch: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("ManipClutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerManipClutch, this, "Button");
    }

    // SUJClutch: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("SUJClutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch, this, "Button");
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        interfaceProvided->AddCommandReadState(this->StateTable, JointCurrentParam, "GetPositionJoint");
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianCurrentParam, "GetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetPositionCartesian, this, "SetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetOpenAngle, this, "SetOpenAngle");

        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotStatusMsg, "RobotStatusMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotErrorMsg, "RobotErrorMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.ManipClutch, "ManipClutchBtn", prmEventButton());
        interfaceProvided->AddEventWrite(EventTriggers.SUJClutch, "SUJClutchBtn", prmEventButton());
        // Stats
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }
}

void mtsIntuitiveResearchKitPSM::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
    }
}

void mtsIntuitiveResearchKitPSM::Startup(void)
{
    this->SetState(PSM_UNINITIALIZED);
    // tool tip transform
    Frame6to7.Assign(0.0, -1.0,  0.0, 0.0,
                     0.0,  0.0,  1.0, 0.0102,
                     -1.0, 0.0,  0.0, 0.0,
                     0.0,  0.0,  0.0, 1.0);
    Frame6to7Inverse = Frame6to7.Inverse();
}

void mtsIntuitiveResearchKitPSM::Run(void)
{
    Counter++;

    ProcessQueuedEvents();
    GetRobotData();

    switch (RobotState) {
    case PSM_UNINITIALIZED:
        break;
    case PSM_HOMING_POWERING:
        RunHomingPower();
        break;
    case PSM_HOMING_CALIBRATING_ARM:
        RunHomingCalibrateArm();
        break;
    case PSM_ARM_CALIBRATED:
        break;
    case PSM_ENGAGING_ADAPTER:
        RunEngagingAdapter();
        break;
    case PSM_ADAPTER_ENGAGED:
        // choose next state
        break;
    case PSM_ENGAGING_TOOL:
        RunEngagingTool();
        break;
    case PSM_READY:
    case PSM_POSITION_CARTESIAN:
        RunPositionCartesian();
        break;
    case PSM_MANUAL:
        break;
    default:
        break;
    }

    RunEvent();
    ProcessQueuedCommands();
}

void mtsIntuitiveResearchKitPSM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitPSM::GetRobotData(void)
{
    // we can start reporting some joint values after the robot is powered
    if (this->RobotState > PSM_HOMING_POWERING) {
        mtsExecutionResult executionResult;
        executionResult = PID.GetPositionJoint(JointCurrentParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
        }
        
        // angles are radians but cisst uses mm.   robManipulator uses SI, so we need meters
        JointCurrentParam.Position().Element(2) /= 1000.0;  // convert from mm to m

        // assign to a more convenient vctDoubleVEc
        JointCurrent.Assign(JointCurrentParam.Position(), NumberOfJoints);

        // when the robot is ready, we can comput cartesian position
        if (this->RobotState >= PSM_READY) {
            // apply tool tip transform
            vctFrm4x4 position;
            position = Manipulator.ForwardKinematics(JointCurrent);
            position = position * Frame6to7;
            position.Rotation().NormalizedSelf();
            CartesianCurrent.Assign(position);
        } else {
            CartesianCurrent.Assign(vctFrm4x4::Identity());
        }
        CartesianCurrentParam.Position().From(CartesianCurrent);
    }
}

void mtsIntuitiveResearchKitPSM::SetState(const RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state " << newState << std::endl;

    switch (newState) {

    case PSM_UNINITIALIZED:
        EventTriggers.RobotStatusMsg(this->GetName() + " not initialized");
        break;

    case PSM_HOMING_POWERING:
        HomingTimer = 0.0;
        HomingPowerRequested = false;
        HomingPowerCurrentBiasRequested = false;
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " powering");
        break;

    case PSM_HOMING_CALIBRATING_ARM:
        HomingCalibrateArmStarted = false;
        RobotState = newState;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " calibrating arm");
        break;

    case PSM_ARM_CALIBRATED:
        RobotState = newState;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " arm calibrated");
        // check if adpater is present and trigger new state
        Adapter.GetButton(Adapter.IsPresent);
        Adapter.IsPresent = !Adapter.IsPresent;
        if (Adapter.IsPresent) {
            SetState(PSM_ENGAGING_ADAPTER);
        }
        break;

    case PSM_ENGAGING_ADAPTER:
        EngagingAdapterStarted = false;
        if (this->RobotState < PSM_ARM_CALIBRATED) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not calibrated yet");
            return;
        }
        // if the tool is present, the adapter is already engadged
        Tool.GetButton(Tool.IsPresent);
        Tool.IsPresent = !Tool.IsPresent;
        if (Tool.IsPresent) {
            SetState(PSM_ADAPTER_ENGAGED);
        } else {
            RobotState = newState;
            this->EventTriggers.RobotStatusMsg(this->GetName() + " engaging adapter");
        }
        break;

    case PSM_ADAPTER_ENGAGED:
        RobotState = newState;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " adapter engaged");
        // check if tool is present and trigger new state
        Tool.GetButton(Tool.IsPresent);
        Tool.IsPresent = !Tool.IsPresent;
        if (Tool.IsPresent) {
            SetState(PSM_ENGAGING_TOOL);
        }
        break;

    case PSM_ENGAGING_TOOL:
        EngagingToolStarted = false;
        if (this->RobotState < PSM_ADAPTER_ENGAGED) {
            EventTriggers.RobotErrorMsg(this->GetName() + " adapter is not engaged yet");
            return;
        }
        RobotState = newState;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " engaging tool");
        break;

    case PSM_READY:
        // when returning from manual mode, need to re-enable PID
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " ready");
        break;

    case PSM_POSITION_CARTESIAN:
        if (this->RobotState < PSM_ARM_CALIBRATED) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not calibrated");
            return;
        }
        // check that the tool is inserted deep enough
        if (JointCurrent.Element(2) < 80.0 / 1000.0) {
            EventTriggers.RobotErrorMsg(this->GetName() + " can't start cartesian mode, make sure the tool is inserted past the cannula");
            break;
        }
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " position cartesian");
        break;

    case PSM_MANUAL:
        if (this->RobotState < PSM_ARM_CALIBRATED) {
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

void mtsIntuitiveResearchKitPSM::RunHomingPower(void)
{
    const double timeToPower = 3.0 * cmn_s;
    const double timeToCalibrateCurrent = 1.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!HomingPowerRequested) {
        RobotIO.BiasEncoder();
        HomingTimer = currentTime;
        // make sure the PID is not sending currents
        PID.Enable(false);
        // pre-load the boards with zero current
        RobotIO.SetMotorCurrent(vctDoubleVec(NumberOfJoints, 0.0));
        // enable power and set a flags to move to next step
        RobotIO.EnablePower();
        HomingPowerRequested = true;
        HomingPowerCurrentBiasRequested = false;
        EventTriggers.RobotStatusMsg(this->GetName() + " power requested");
        return;
    }

    // second, request current bias, we leave some time for power to stabilize
    if (!HomingPowerCurrentBiasRequested
        && ((currentTime - HomingTimer) > timeToPower)) {
        HomingTimer = currentTime;
        RobotIO.BiasCurrent(500); // 500 samples, this API should be changed to use time
        HomingPowerCurrentBiasRequested = true;
        return;
    }

    // wait some more to be ready
    if (HomingPowerCurrentBiasRequested
        && (currentTime - HomingTimer) > timeToCalibrateCurrent) {
        vctBoolVec amplifiersStatus(NumberOfJoints);
        RobotIO.GetAmpStatus(amplifiersStatus);
        if (amplifiersStatus.All()) {
            EventTriggers.RobotStatusMsg(this->GetName() + " power on");
            this->SetState(PSM_HOMING_CALIBRATING_ARM);
        } else {
            EventTriggers.RobotErrorMsg(this->GetName() + " failed to enable power.");
            this->SetState(PSM_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitPSM::RunHomingCalibrateArm(void)
{
    static const double timeToHome = 2.0 * cmn_s;
    static const double extraTime = 5.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    // trigger motion
    if (!HomingCalibrateArmStarted) {
        // disable joint limits
        PID.SetCheckJointLimit(false);
        // enable PID and start from current position
        JointDesired.ForceAssign(JointCurrent);
        SetPositionJointLocal(JointDesired);
        PID.Enable(true);

        // compute joint goal position
        JointTrajectory.Goal.SetSize(NumberOfJoints);
        JointTrajectory.Goal.ForceAssign(JointCurrent);
        JointTrajectory.Goal.Element(0) = 0.0;
        JointTrajectory.Goal.Element(1) = 0.0;
        JointTrajectory.Goal.Element(2) = 0.0;
        JointTrajectory.Quintic.Set(currentTime,
                                    JointCurrent, JointTrajectory.Zero, JointTrajectory.Zero,
                                    currentTime + timeToHome,
                                    JointTrajectory.Goal, JointTrajectory.Zero, JointTrajectory.Zero);
        HomingTimer = JointTrajectory.Quintic.StopTime();
        // set flag to indicate that homing has started
        HomingCalibrateArmStarted = true;
    }

    // compute a new set point based on time
    if (currentTime <= HomingTimer) {
        JointTrajectory.Quintic.Evaluate(currentTime, JointDesired,
                                         JointTrajectory.Velocity, JointTrajectory.Acceleration);
        SetPositionJointLocal(JointDesired);
    } else {
        // request final position in case trajectory rounding prevent us to get there
        SetPositionJointLocal(JointTrajectory.Goal);

        // check position
        JointTrajectory.GoalError.DifferenceOf(JointTrajectory.Goal, JointCurrent);
        JointTrajectory.GoalError.AbsSelf();
        bool isHomed = !JointTrajectory.GoalError.ElementwiseGreaterOrEqual(JointTrajectory.GoalTolerance).Any();
        if (isHomed) {
            PID.SetCheckJointLimit(true);
            EventTriggers.RobotStatusMsg(this->GetName() + " arm calibrated");
            this->SetState(PSM_ARM_CALIBRATED);
        } else {
            // time out
            if (currentTime > HomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingCalibrateArm: unable to reach home position, error in degrees is "
                                           << JointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                EventTriggers.RobotErrorMsg(this->GetName() + " unable to reach home position during calibration on pots.");
                PID.Enable(false);
                PID.SetCheckJointLimit(true);
                this->SetState(PSM_UNINITIALIZED);
            }
        }
    }
}

void mtsIntuitiveResearchKitPSM::RunEngagingAdapter(void)
{
    if (!EngagingAdapterStarted) {
        EngagingStopwatch.Reset();
        EngagingStopwatch.Start();
        // get current joint desired position from PID
        PID.GetPositionJointDesired(JointDesired);
        EngagingJointSet.ForceAssign(JointDesired);
        // disable joint limits
        PID.SetCheckJointLimit(false);
        EngagingAdapterStarted = true;
        return;
    }

    // PSM tool last 4 actuator coupling matrix
    // psm_m2jpos = [-1.5632  0.0000  0.0000  0.0000;
    //                0.0000  1.0186  0.0000  0.0000;
    //                0.0000 -0.8306  0.6089  0.6089;
    //                0.0000  0.0000 -1.2177  1.2177];
    // each actuator has -180 to 180 deg limits
    // these joint limit is computed as
    // joint_lim = psm_m2jpos * actuator_lim

    if (EngagingStopwatch.GetElapsedTime() > (3500 * cmn_ms)){
        EngagingJointSet[3] = 0.0;
        EngagingJointSet[4] = 0.0;
        EngagingJointSet[5] = 0.0;
        EngagingJointSet[6] = 0.0;
        JointDesired.ForceAssign(EngagingJointSet);
        PID.SetCheckJointLimit(true);
        SetPositionJointLocal(JointDesired);

        // Adapter engage done
        EngagingStopwatch.Reset();
        SetState(PSM_ADAPTER_ENGAGED);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (2500 * cmn_ms)){
        EngagingJointSet[3] = -300.0 * cmnPI / 180.0;
        EngagingJointSet[4] =  170.0 * cmnPI / 180.0;
        EngagingJointSet[5] =   65.0 * cmnPI / 180.0;
        EngagingJointSet[6] =    0.0 * cmnPI / 180.0;
        JointDesired.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointDesired);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (1500 * cmn_ms)){
        EngagingJointSet[3] =  300.0 * cmnPI / 180.0;
        EngagingJointSet[4] = -170.0 * cmnPI / 180.0;
        EngagingJointSet[5] =  -65.0 * cmnPI / 180.0;
        EngagingJointSet[6] =    0.0 * cmnPI / 180.0;
        JointDesired.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointDesired);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (500 * cmn_ms)){
        EngagingJointSet[3] = -300.0 * cmnPI / 180.0;
        EngagingJointSet[4] =  170.0 * cmnPI / 180.0;
        EngagingJointSet[5] =   65.0 * cmnPI / 180.0;
        EngagingJointSet[6] =    0.0 * cmnPI / 180.0;
        JointDesired.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointDesired);
    }
}

void mtsIntuitiveResearchKitPSM::RunEngagingTool(void)
{
    if (!EngagingToolStarted) {
        EngagingStopwatch.Reset();
        EngagingStopwatch.Start();
        // get current joint desired position from PID
        PID.GetPositionJointDesired(JointDesired);
        EngagingJointSet.ForceAssign(JointDesired);
        // disable joint limits
        PID.SetCheckJointLimit(false);
        EngagingToolStarted = true;
        return;
    }

    if (EngagingStopwatch.GetElapsedTime() > (2500 * cmn_ms)){
        // get current joint desired position from PID
        PID.GetPositionJointDesired(JointDesired);
        // open gripper
        JointDesired[6] = 10.0 * cmnPI / 180.0;
        PID.SetCheckJointLimit(true);
        SetPositionJointLocal(JointDesired);

        // Adapter engage done
        EngagingStopwatch.Reset();
        SetState(PSM_READY);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (2000 * cmn_ms)){
        EngagingJointSet[3] = -280.0 * cmnPI / 180.0;
        EngagingJointSet[4] =  10.0 * cmnPI / 180.0;
        EngagingJointSet[5] =  10.0 * cmnPI / 180.0;
        EngagingJointSet[6] =  10.0 * cmnPI / 180.0;
        JointDesired.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointDesired);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (1500 * cmn_ms)){
        EngagingJointSet[3] = -280.0 * cmnPI / 180.0;
        EngagingJointSet[4] =  10.0 * cmnPI / 180.0;
        EngagingJointSet[5] = -10.0 * cmnPI / 180.0;
        EngagingJointSet[6] =  10.0 * cmnPI / 180.0;
        JointDesired.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointDesired);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (1000 * cmn_ms)){
        EngagingJointSet[3] =  280.0 * cmnPI / 180.0;
        EngagingJointSet[4] = -10.0 * cmnPI / 180.0;
        EngagingJointSet[5] =  10.0 * cmnPI / 180.0;
        EngagingJointSet[6] =  10.0 * cmnPI / 180.0;
        JointDesired.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointDesired);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (500 * cmn_ms)){
        EngagingJointSet[3] = -280.0 * cmnPI / 180.0;
        EngagingJointSet[4] = -10.0 * cmnPI / 180.0;
        EngagingJointSet[5] = -10.0 * cmnPI / 180.0;
        EngagingJointSet[6] =  10.0 * cmnPI / 180.0;
        JointDesired.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointDesired);
    }

}

void mtsIntuitiveResearchKitPSM::RunPositionCartesian(void)
{
    // should prevent user to go to close to RCM
}

void mtsIntuitiveResearchKitPSM::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    JointDesiredParam.Goal().Assign(newPosition, NumberOfJoints);
    JointDesiredParam.Goal().Element(2) *= 1000.0; // convert from meters to mm
    JointDesiredParam.Goal().Element(7) = 0.0;
    PID.SetPositionJoint(JointDesiredParam);
}

void mtsIntuitiveResearchKitPSM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (RobotState == PSM_POSITION_CARTESIAN) {
        vctDoubleVec jointSet(6, 0.0);
        jointSet.Assign(JointCurrent, 6);

        // compute desired slave position
        CartesianPositionFrm.From(newPosition.Goal());
        CartesianPositionFrm = CartesianPositionFrm * Frame6to7Inverse;

        Manipulator.InverseKinematics(jointSet, CartesianPositionFrm);
        jointSet.resize(7);
        jointSet[6] = DesiredOpenAngle;

        // deals with J4: outer roll
        double j4compensate = (jointSet[3] - JointCurrent[3])/(2.0*cmnPI);
        if (j4compensate >= 0) {
            j4compensate = (int)(j4compensate + 0.5);
        } else {
            j4compensate = (int)(j4compensate - 0.5);
        }
        jointSet[3] -= (j4compensate * cmnPI * 2.0);

        /*
        if (Counter%5) {
            CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": cur = " << JointCurrent[3]
                                    << " cmd = " << jointSet[3] + j4compensate * cmnPI * 2.0
                                    << " cmdfix = " << jointSet[3] << std::endl;
        }
*/

        // note: this directly calls the lower level to set position,
        // maybe we should cache the request in this component and later
        // in the Run method push the request.  This way, only the latest
        // request would be pushed if multiple are queued.
        SetPositionJointLocal(jointSet);
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionCartesian: PSM not ready" << std::endl;
    }
}

void mtsIntuitiveResearchKitPSM::SetOpenAngle(const double & openAngle)
{
    DesiredOpenAngle = openAngle;
}

void mtsIntuitiveResearchKitPSM::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(PSM_HOMING_POWERING);
    } else if ((state == "Cartesian position") || (state == "Teleop")) {
        SetState(PSM_POSITION_CARTESIAN);
    } else if (state == "Manual") {
        SetState(PSM_MANUAL);
    } else {
        EventTriggers.RobotErrorMsg(this->GetName() + ": unsupported state " + state);
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerAdapter(const prmEventButton &button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        SetState(PSM_ENGAGING_ADAPTER);
    } else {
        // this is "down" transition so we have to
        // make sure we had an adapter properly engaged before
        if (RobotState >= PSM_ADAPTER_ENGAGED) {
            SetState(PSM_ARM_CALIBRATED);
        }
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerTool(const prmEventButton &button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        SetState(PSM_ENGAGING_TOOL);
    } else {
        // this is "down" transition so we have to
        // make sure we had a tool properly engaged before
        if (RobotState >= PSM_READY) {
            SetState(PSM_ADAPTER_ENGAGED);
        }
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerManipClutch(const prmEventButton &button)
{
    // Pass events
    EventTriggers.ManipClutch(button);

    // Start manual mode but save the previous state
    if (button.Type() == prmEventButton::PRESSED) {
        EventTriggers.ManipClutchPreviousState = this->RobotState;
        SetState(PSM_MANUAL);
    } else {
        if (RobotState == PSM_MANUAL) {
            // Enable PID
            PID.Enable(true);
            // set command joint position to joint current
            JointDesired.ForceAssign(JointCurrent);
            SetPositionJointLocal(JointDesired);
            // go back to state before clutching
            SetState(EventTriggers.ManipClutchPreviousState);

        }
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch(const prmEventButton &button)
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

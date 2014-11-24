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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>

#define TRAJECTORY_FOR_POSITION_CARTESIAN 0 // adeguet1, testing trajectory for all cartesian set position

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
    IsCartesianGoalSet = false;
    Counter = 0;

    SetState(PSM_UNINITIALIZED);
    DesiredOpenAngle = 0.0 * cmnPI_180;

    // initialize trajectory data
    JointGet.SetSize(NumberOfJoints);
    JointSet.SetSize(NumberOfJoints);
    JointSetParam.Goal().SetSize(NumberOfJoints);
    JointTrajectory.Velocity.SetSize(NumberOfJoints);
    JointTrajectory.Velocity.SetAll(360.0 * cmnPI_180); // degrees per second
    JointTrajectory.Velocity.Element(2) = 0.2; // m per second
    JointTrajectory.Acceleration.SetSize(NumberOfJoints);
    JointTrajectory.Acceleration.SetAll(360.0 * cmnPI_180);
    JointTrajectory.Acceleration.Element(2) = 0.2; // m per second
    JointTrajectory.Start.SetSize(NumberOfJoints);
    JointTrajectory.Goal.SetSize(NumberOfJoints);
    JointTrajectory.GoalError.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI / 180.0); // hard coded to 3 degrees
    JointTrajectory.EndTime = 0.0;
    EngagingJointSet.SetSize(NumberOfJoints);

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
    }

    // Robot IO
    interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("EnablePower", RobotIO.EnablePower);
        interfaceRequired->AddFunction("DisablePower", RobotIO.DisablePower);
        interfaceRequired->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        interfaceRequired->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
        interfaceRequired->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        interfaceRequired->AddFunction("UsePotsForSafetyCheck", RobotIO.UsePotsForSafetyCheck);
        interfaceRequired->AddFunction("SetPotsToEncodersTolerance", RobotIO.SetPotsToEncodersTolerance);
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
        interfaceProvided->AddCommandReadState(this->StateTable, JointGetParam, "GetPositionJoint");
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianGetParam, "GetPositionCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianGetDesiredParam, "GetPositionCartesianDesired");
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

    // Initialize the optimizer
    Optimizer = new mtsIntuitiveResearchKitOptimizer(6);
    Optimizer->InitializeFollowVF(6,
                                  "FollowVFSlave",
                                  "CurrentSlaveKinematics",
                                  "DesiredSlaveKinematics");
}

void mtsIntuitiveResearchKitPSM::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
    } else {
        // tool tip transform, this should come from a configuration file
        ToolOffsetTransformation.Assign(0.0, -1.0,  0.0, 0.0,
                                        0.0,  0.0,  1.0, 0.0,
                                        -1.0, 0.0,  0.0, 0.0,
                                        0.0,  0.0,  0.0, 1.0);
        ToolOffset = new robManipulator(ToolOffsetTransformation);
        Manipulator.Attach(ToolOffset);
    }
}

void mtsIntuitiveResearchKitPSM::Startup(void)
{
    this->SetState(PSM_UNINITIALIZED);
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
        break;
    case PSM_POSITION_CARTESIAN:
        RunPositionCartesian();
        break;
    case PSM_CONSTRAINT_CONTROLLER_CARTESIAN:
        RunConstraintControllerCartesian();
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
        // actual joints
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
        if (this->RobotState >= PSM_READY) {
            // update cartesian position
            CartesianGet = Manipulator.ForwardKinematics(JointGet);
            CartesianGet.Rotation().NormalizedSelf();
            // update cartesian position desired based on joint desired
            CartesianGetDesired = Manipulator.ForwardKinematics(JointGetDesired);
            CartesianGetDesired.Rotation().NormalizedSelf();
        } else {
            CartesianGet.Assign(vctFrm4x4::Identity());
            CartesianGetDesired.Assign(vctFrm4x4::Identity());
        }
        CartesianGetParam.Position().From(CartesianGet);
        CartesianGetDesiredParam.Position().From(CartesianGetDesired);
    }
}

void mtsIntuitiveResearchKitPSM::SetState(const RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state " << newState << std::endl;

    switch (newState) {

    case PSM_UNINITIALIZED:
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " not initialized");
        break;

    case PSM_HOMING_POWERING:
        HomingTimer = 0.0;
        HomingPowerRequested = false;
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
            EventTriggers.RobotStatusMsg(this->GetName() + " is not calibrated yet, will engage adapter later");
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
            EventTriggers.RobotStatusMsg(this->GetName() + " adapter is not engaged yet, will engage tool later");
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
        if (JointGet.Element(2) < 80.0 / 1000.0) {
            EventTriggers.RobotErrorMsg(this->GetName() + " can't start cartesian mode, make sure the tool is inserted past the cannula");
            break;
        }
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " position cartesian");
        break;

    case PSM_CONSTRAINT_CONTROLLER_CARTESIAN:
        if (this->RobotState < PSM_ARM_CALIBRATED) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not calibrated");
            return;
        }
        // check that the tool is inserted deep enough
        if (JointGet.Element(2) < 80.0 / 1000.0) {
            EventTriggers.RobotErrorMsg(this->GetName() + " can't start constraint controller cartesian mode, make sure the tool is inserted past the cannula");
            break;
        }
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " constraint controller cartesian");
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

    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!HomingPowerRequested) {
        RobotIO.BiasEncoder();
        if (0) { // use pots for redundancy
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
        vctBoolVec amplifiersStatus(NumberOfJoints);
        RobotIO.GetActuatorAmpStatus(amplifiersStatus);
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
    static const double extraTime = 5.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    // trigger motion
    if (!HomingCalibrateArmStarted) {
        // disable joint limits
        PID.SetCheckJointLimit(false);
        // enable PID and start from current position
        JointSet.ForceAssign(JointGet);
        SetPositionJointLocal(JointSet);
        PID.Enable(true);

        // compute joint goal position
        JointTrajectory.Goal.SetSize(NumberOfJoints);
        JointTrajectory.Goal.ForceAssign(JointGet);
        // move to zero position only there is no tool present
        Tool.GetButton(Tool.IsPresent);
        Tool.IsPresent = !Tool.IsPresent;
        if (!Tool.IsPresent) {
            JointTrajectory.Goal.Element(0) = 0.0;
            JointTrajectory.Goal.Element(1) = 0.0;
            JointTrajectory.Goal.Element(2) = 0.0;
        }
        JointTrajectory.LSPB.Set(JointGet, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime - this->GetPeriodicity(), robLSPB::LSPB_DURATION);
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
        EngagingJointSet.ForceAssign(JointGetDesired);
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

    if (EngagingStopwatch.GetElapsedTime() > (3500 * cmn_ms)) {
        EngagingJointSet[3] = 0.0;
        EngagingJointSet[4] = 0.0;
        EngagingJointSet[5] = 0.0;
        EngagingJointSet[6] = 0.0;
        JointSet.ForceAssign(EngagingJointSet);
        PID.SetCheckJointLimit(true);
        SetPositionJointLocal(JointSet);

        // Adapter engage done
        EngagingStopwatch.Reset();
        SetState(PSM_ADAPTER_ENGAGED);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (2500 * cmn_ms)) {
        EngagingJointSet[3] = -300.0 * cmnPI / 180.0;
        EngagingJointSet[4] =  170.0 * cmnPI / 180.0;
        EngagingJointSet[5] =   65.0 * cmnPI / 180.0;
        EngagingJointSet[6] =    0.0 * cmnPI / 180.0;
        JointSet.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointSet);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (1500 * cmn_ms)) {
        EngagingJointSet[3] =  300.0 * cmnPI / 180.0;
        EngagingJointSet[4] = -170.0 * cmnPI / 180.0;
        EngagingJointSet[5] =  -65.0 * cmnPI / 180.0;
        EngagingJointSet[6] =    0.0 * cmnPI / 180.0;
        JointSet.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointSet);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (500 * cmn_ms)) {
        EngagingJointSet[3] = -300.0 * cmnPI / 180.0;
        EngagingJointSet[4] =  170.0 * cmnPI / 180.0;
        EngagingJointSet[5] =   65.0 * cmnPI / 180.0;
        EngagingJointSet[6] =    0.0 * cmnPI / 180.0;
        JointSet.ForceAssign(EngagingJointSet);
        SetPositionJointLocal(JointSet);
    }
}

void mtsIntuitiveResearchKitPSM::RunEngagingTool(void)
{
    if (!EngagingToolStarted) {
        EngagingStopwatch.Reset();
        EngagingStopwatch.Start();
        EngagingJointSet.ForceAssign(JointGetDesired);
        // disable joint limits
        PID.SetCheckJointLimit(false);
        EngagingToolStarted = true;
        return;
    }

    if (EngagingStopwatch.GetElapsedTime() > (2500 * cmn_ms)) {
        // straight wrist open gripper
        EngagingJointSet[3] = 0.0;
        EngagingJointSet[4] = 0.0;
        EngagingJointSet[5] = 0.0;
        EngagingJointSet[6] = 10.0 * cmnPI / 180.0;
        PID.SetCheckJointLimit(true);
        JointSet.Assign(EngagingJointSet);
        SetPositionJointLocal(JointSet);
        // Adapter engage done
        EngagingStopwatch.Reset();
        SetState(PSM_READY);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (2000 * cmn_ms)) {
        EngagingJointSet[3] = -280.0 * cmnPI / 180.0;
        EngagingJointSet[4] =  10.0 * cmnPI / 180.0;
        EngagingJointSet[5] =  10.0 * cmnPI / 180.0;
        EngagingJointSet[6] =  10.0 * cmnPI / 180.0;
        JointSet.Assign(EngagingJointSet);
        SetPositionJointLocal(JointSet);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (1500 * cmn_ms)) {
        EngagingJointSet[3] = -280.0 * cmnPI / 180.0;
        EngagingJointSet[4] =  10.0 * cmnPI / 180.0;
        EngagingJointSet[5] = -10.0 * cmnPI / 180.0;
        EngagingJointSet[6] =  10.0 * cmnPI / 180.0;
        JointSet.Assign(EngagingJointSet);
        SetPositionJointLocal(JointSet);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (1000 * cmn_ms)) {
        EngagingJointSet[3] =  280.0 * cmnPI / 180.0;
        EngagingJointSet[4] = -10.0 * cmnPI / 180.0;
        EngagingJointSet[5] =  10.0 * cmnPI / 180.0;
        EngagingJointSet[6] =  10.0 * cmnPI / 180.0;
        JointSet.Assign(EngagingJointSet);
        SetPositionJointLocal(JointSet);
    }
    else if (EngagingStopwatch.GetElapsedTime() > (500 * cmn_ms)) {
        EngagingJointSet[3] = -280.0 * cmnPI / 180.0;
        EngagingJointSet[4] = -10.0 * cmnPI / 180.0;
        EngagingJointSet[5] = -10.0 * cmnPI / 180.0;
        EngagingJointSet[6] =  10.0 * cmnPI / 180.0;
        JointSet.Assign(EngagingJointSet);
        SetPositionJointLocal(JointSet);
    }
}

void mtsIntuitiveResearchKitPSM::RunPositionCartesian(void)
{
    //! \todo: should prevent user to go to close to RCM!

    // sanity check
    if (RobotState != PSM_POSITION_CARTESIAN) {
        CMN_LOG_CLASS_RUN_ERROR << GetName() << ": SetPositionCartesian: PSM not ready" << std::endl;
        return;
    }

#if TRAJECTORY_FOR_POSITION_CARTESIAN
    const double currentTime = this->StateTable.GetTic();
    if (currentTime <= JointTrajectory.EndTime) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    }
#else
    if (IsCartesianGoalSet == true) {
        vctDoubleVec jointSet(6, 0.0);
        jointSet.Assign(JointGetDesired, 6);

        // compute desired slave position
        CartesianPositionFrm.From(CartesianSetParam.Goal());
        Manipulator.InverseKinematics(jointSet, CartesianPositionFrm);
        jointSet.resize(7);
        jointSet[6] = DesiredOpenAngle;

        // find closest solution mod 2 pi
        const double difference = JointGetDesired[3] - jointSet[3];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[3] = jointSet[3] + differenceInTurns * 2.0 * cmnPI;

        // finally send new joint values
        SetPositionJointLocal(jointSet);
    }
#endif
}

void mtsIntuitiveResearchKitPSM::RunConstraintControllerCartesian(void)
{
    // Update the optimizer
    // Go through the VF list, update state data pointers, assign tableau references, and fill in the references
    if (IsCartesianGoalSet) {
        IsCartesianGoalSet = false;

        // Update kinematics and VF data objects
        Optimizer->UpdateParams(JointGet,
                                   Manipulator,
                                   this->GetPeriodicity(),
                                   CartesianGet,
                                   vctFrm4x4(CartesianSetParam.Goal())
                                   );

        vctDoubleVec dq;
        // Make sure the return value is meaningful
        if (Optimizer->Solve(dq)) {
            // make appropriate adjustments to incremental motion specific to davinci

            // send command to move to specified position
            vctDoubleVec FinalJoint(6);
            FinalJoint.Assign(JointGet,6);
            FinalJoint = FinalJoint + dq;
            FinalJoint.resize(7);
            FinalJoint[6] = DesiredOpenAngle;

            // find closest solution mod 2 pi
            double diffTurns = nearbyint(-dq[3] / (2.0 * cmnPI));
            FinalJoint[3] = FinalJoint[3] + diffTurns * 2.0 * cmnPI;

            // Send the final joint commands to the LLC
            SetPositionJointLocal(FinalJoint);
        }
        else {
            CMN_LOG_CLASS_RUN_ERROR << "Control Optimizer failed " << std::endl;
        }
    }
}

void mtsIntuitiveResearchKitPSM::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    CMN_LOG_CLASS_RUN_DEBUG << "SetPositionJointLocal: " << newPosition << std::endl;
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints);
    JointSetParam.Goal().Element(2) *= 1000.0; // convert from meters to mm
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitPSM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (RobotState == PSM_POSITION_CARTESIAN) {
#if TRAJECTORY_FOR_POSITION_CARTESIAN
        // first compute inverse kinematics on first 6 joints
        vctDoubleVec joints(6);
        joints.Assign(JointGet, 6);
        Manipulator.InverseKinematics(joints, newPosition.Goal());
        // find closest solution mod 2 pi for rotation along tool shaft
        const double difference = JointGet[3] - joints[3];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        joints[3] = joints[3] + differenceInTurns * 2.0 * cmnPI;

        // trajectory in joint space
        const double currentTime = this->StateTable.GetTic();
        // starting point is last requested to PID component
        JointTrajectory.Start.Assign(JointGet);
        // assign inverse kinematics joints and angle to goal
        JointTrajectory.Goal.Assign(joints, 6);
        JointTrajectory.Goal.Element(6) = DesiredOpenAngle;
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime - this->GetPeriodicity(), robLSPB::LSPB_NONE); // LSPB_DURATION);
        JointTrajectory.EndTime = currentTime + JointTrajectory.LSPB.Duration();
#else
        CartesianSetParam = newPosition;
        IsCartesianGoalSet = true;
#endif
    } else if (RobotState == PSM_CONSTRAINT_CONTROLLER_CARTESIAN) {
        CartesianSetParam = newPosition;
        IsCartesianGoalSet = true;
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
    } else if (state == "Cartesian constraint controller") {
        SetState(PSM_CONSTRAINT_CONTROLLER_CARTESIAN);
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
            JointSet.ForceAssign(JointGet);
            SetPositionJointLocal(JointSet);
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

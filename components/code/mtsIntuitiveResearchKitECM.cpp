/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitECM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitECM::mtsIntuitiveResearchKitECM(const std::string & componentName, const double periodInSeconds):
    mtsIntuitiveResearchKitArm(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitECM::mtsIntuitiveResearchKitECM(const mtsTaskPeriodicConstructorArg & arg):
    mtsIntuitiveResearchKitArm(arg)
{
    Init();
}

void mtsIntuitiveResearchKitECM::SetSimulated(void)
{
    mtsIntuitiveResearchKitArm::SetSimulated();
    // in simulation mode, we don't need clutch IO
    RemoveInterfaceRequired("ManipClutch");
}

robManipulator::Errno mtsIntuitiveResearchKitECM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal)
{
    // re-align desired frame to 4 axis direction to reduce free space
    vctDouble3 shaft = cartesianGoal.Translation();
    shaft.NormalizedSelf();
    const vctDouble3 z = cartesianGoal.Rotation().Column(2).Ref<3>(); // last column of rotation matrix
    vctDouble3 axis;
    axis.CrossProductOf(z, shaft);
    const double angle = acos(vctDotProduct(z, shaft));

    const vctMatRot3 reAlign(vctAxAnRot3(axis, angle, VCT_NORMALIZE));
    vctFrm4x4 newGoal;
    newGoal.Translation().Assign(cartesianGoal.Translation());
    newGoal.Rotation().ProductOf(reAlign, cartesianGoal.Rotation());

    if (Manipulator.InverseKinematics(jointSet, newGoal) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = StateJoint.Position()[3] - jointSet[3];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[3] = jointSet[3] + differenceInTurns * 2.0 * cmnPI;
        // make sure we are away from RCM point, this test is
        // simplistic
        if (jointSet[2] < 40.0 * cmn_mm) {
            jointSet[2] = 40.0 * cmn_mm;
        }
#if 0
        vctFrm4x4 forward = Manipulator.ForwardKinematics(jointSet);
        vctDouble3 diff;
        diff.DifferenceOf(forward.Translation(), newGoal.Translation());
        std::cerr << cmnInternalTo_mm(diff.Norm()) << "mm ";
#endif
        return robManipulator::ESUCCESS;
    }
    return robManipulator::EFAILURE;
}

void mtsIntuitiveResearchKitECM::Init(void)
{
    // main initialization from base type
    mtsIntuitiveResearchKitArm::Init();

    // initialize trajectory data
    JointTrajectory.Velocity.Assign(30.0 * cmnPI_180, // degrees per second
                                    30.0 * cmnPI_180,
                                     0.05,            // m per second
                                    20.0 * cmnPI_180);
    JointTrajectory.Acceleration.Assign(30.0 * cmnPI_180,
                                        30.0 * cmnPI_180,
                                         0.05,
                                        30.0 * cmnPI_180);
    JointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI / 180.0); // hard coded to 3 degrees
    PotsToEncodersTolerance.SetAll(10.0 * cmnPI_180); // 10 degrees for rotations
    PotsToEncodersTolerance.Element(2) = 20.0 * cmn_mm; // 20 mm

    mtsInterfaceRequired * interfaceRequired;

    // Main interface should have been created by base class init
    CMN_ASSERT(RobotInterface);
    RobotInterface->AddEventWrite(ClutchEvents.ManipClutch, "ManipClutch", prmEventButton());

    // ManipClutch: digital input button event from ECM
    interfaceRequired = AddInterfaceRequired("ManipClutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECM::EventHandlerManipClutch, this, "Button");
    }
}


void mtsIntuitiveResearchKitECM::Configure(const std::string & filename)
{
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": failed to parse configuration\n"
                                     << jsonReader.getFormattedErrorMessages();
            return;
        }

        ConfigureDH(jsonConfig);

        // should arm go to zero position when homing, default set in Init method
        const Json::Value jsonHomingGoesToZero = jsonConfig["homing-zero-position"];
        if (!jsonHomingGoesToZero.isNull()) {
            HomingGoesToZero = jsonHomingGoesToZero.asBool();
        }

        // load tool tip transform if any (for up/down endoscopes)
        const Json::Value jsonToolTip = jsonConfig["tooltip-offset"];
        if (!jsonToolTip.isNull()) {
            cmnDataJSON<vctFrm4x4>::DeSerializeText(ToolOffsetTransformation, jsonToolTip);
            ToolOffset = new robManipulator(ToolOffsetTransformation);
            Manipulator.Attach(ToolOffset);
        }
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
    }
}


void mtsIntuitiveResearchKitECM::SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state "
                            << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(newState) << std::endl;

    // first cleanup from previous state
    switch (RobotState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
        TrajectoryIsUsed(false);
        break;

    default:
        break;
    }

    switch (newState) {

    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
        RobotIO.DisablePower();
        PID.Enable(false);
        PID.SetCheckJointLimit(true);
        TrajectoryIsUsed(false);
        RobotState = newState;
        RobotInterface->SendStatus(this->GetName() + " not initialized");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER:
        HomingBiasEncoderRequested = false;
        RobotState = newState;
        RobotInterface->SendStatus(this->GetName() + " updating encoders based on potentiometers");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING:
        HomingTimer = 0.0;
        HomingPowerRequested = false;
        RobotState = newState;
        RobotInterface->SendStatus(this->GetName() + " powering");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM:
        HomingCalibrateArmStarted = false;
        RobotState = newState;
        this->RobotInterface->SendStatus(this->GetName() + " calibrating arm");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_READY:
        // when returning from manual mode, need to re-enable PID
        RobotState = newState;
        RobotInterface->SendStatus(this->GetName() + " ready");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT:
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            RobotInterface->SendError(this->GetName() + " is not ready");
            return;
        }
        RobotState = newState;
        JointSet.Assign(StateJointDesired.Position(), this->NumberOfJoints());
        if (newState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT) {
            IsGoalSet = false;
            RobotInterface->SendStatus(this->GetName() + " position joint");
        } else {
            TrajectoryIsUsed(true);
            RobotInterface->SendStatus(this->GetName() + " position goal joint");
        }
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED) {
            RobotInterface->SendError(this->GetName() + " is not calibrated");
            return;
        }
        // check that the tool is inserted deep enough
        if (StateJoint.Position().Element(2) < 40.0 * cmn_mm) {
            RobotInterface->SendError(this->GetName() + " can't start cartesian mode, make sure the endoscope is inserted past the cannula (joint 3 > 40 mm)");
        } else {
            if (StateJoint.Position().Element(2) < 50.0 * cmn_mm) {
                RobotInterface->SendWarning(this->GetName() + " cartesian mode started close to RCM (joint 3 < 50 mm), joint 3 will be clamped at 40 mm to avoid moving inside cannula.");
            }
            RobotState = newState;
            if (newState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN) {
                IsGoalSet = false;
                RobotInterface->SendStatus(this->GetName() + " position cartesian");
            } else {
                TrajectoryIsUsed(true);
                RobotInterface->SendStatus(this->GetName() + " position goal cartesian");
            }
        }
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN:
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            RobotInterface->SendError(this->GetName() + " is not ready");
            return;
        }
        // check that the tool is inserted deep enough
        if (StateJoint.Position().Element(2) < 80.0 * cmn_mm) {
            RobotInterface->SendError(this->GetName() + " can't start constraint controller cartesian mode, make sure the tool is inserted past the cannula");
            break;
        }
        RobotState = newState;
        IsGoalSet = false;
        RobotInterface->SendStatus(this->GetName() + " constraint controller cartesian");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL:
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            RobotInterface->SendError(this->GetName() + " is not ready yet");
            return;
        }
        // disable PID to allow manual move
        PID.Enable(false);
        RobotState = newState;
        RobotInterface->SendStatus(this->GetName() + " in manual mode");
        break;
    default:
        break;
    }

    // Emit event with current state
    MessageEvents.RobotState(mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(this->RobotState));
}

void mtsIntuitiveResearchKitECM::RunHomingCalibrateArm(void)
{
    if (mIsSimulated) {
        this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_READY);
        return;
    }

    static const double extraTime = 5.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    // trigger motion
    if (!HomingCalibrateArmStarted) {
        // disable joint limits
        PID.SetCheckJointLimit(false);
        // enable PID and start from current position
        JointSet.ForceAssign(StateJoint.Position());
        SetPositionJointLocal(JointSet);
        // configure PID to fail in case of tracking error
        vctDoubleVec tolerances(NumberOfJoints());
        tolerances.SetAll(7.0 * cmnPI_180); // 7 degrees on angles
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        PID.SetTrackingErrorTolerance(tolerances);
        PID.EnableTrackingError(true);
        // finally enable PID
        PID.Enable(true);

        // compute joint goal position
        JointTrajectory.Goal.SetSize(NumberOfJoints());
        if (HomingGoesToZero) {
            // move to zero position
            JointTrajectory.Goal.SetAll(0.0);
        } else {
            // stay at current position by default
            JointTrajectory.Goal.Assign(StateJoint.Position());
        }

        JointTrajectory.GoalVelocity.SetAll(0.0);
        JointTrajectory.EndTime = 0.0;
        TrajectoryIsUsed(true);
        HomingCalibrateArmStarted = true;
    }

    // compute a new set point based on time
    JointTrajectory.Reflexxes.Evaluate(JointSet,
                                       JointVelocitySet,
                                       JointTrajectory.Goal,
                                       JointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);

    const robReflexxes::ResultType trajectoryResult = JointTrajectory.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (JointTrajectory.EndTime == 0.0) {
            JointTrajectory.EndTime = currentTime + JointTrajectory.Reflexxes.Duration();
            HomingTimer = JointTrajectory.EndTime;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        {
            // check position
            JointTrajectory.GoalError.DifferenceOf(JointTrajectory.Goal, StateJoint.Position());
            JointTrajectory.GoalError.AbsSelf();
            bool isHomed = !JointTrajectory.GoalError.ElementwiseGreaterOrEqual(JointTrajectory.GoalTolerance).Any();
            if (isHomed) {
                PID.SetCheckJointLimit(true);
                RobotInterface->SendStatus(this->GetName() + " arm ready");
                HomedOnce = true;
                this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_READY);
            } else {
                // time out
                if (currentTime > HomingTimer + extraTime) {
                    CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingCalibrateArm: unable to reach home position, error in degrees is "
                                               << JointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                    RobotInterface->SendError(this->GetName() + " unable to reach home position during calibration on pots.");
                    this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
                }
            }
        }
        break;

    default:
        RobotInterface->SendError(this->GetName() + " error while evaluating trajectory.");
        break;
    }
}

void mtsIntuitiveResearchKitECM::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER);
    } else if ((state == "Cartesian position") || (state == "Teleop")) {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN);
    } else if (state == "Manual") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL);
    } else {
        mtsIntuitiveResearchKitArmTypes::RobotStateType stateEnum;
        try {
            stateEnum = mtsIntuitiveResearchKitArmTypes::RobotStateTypeFromString(state);
        } catch (std::exception e) {
            RobotInterface->SendError(this->GetName() + ": ECM unsupported state " + state + ": " + e.what());
            return;
        }
        SetState(stateEnum);
    }
}

void mtsIntuitiveResearchKitECM::EventHandlerTrackingError(void)
{
    RobotInterface->SendError(this->GetName() + ": PID tracking error");
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}

void mtsIntuitiveResearchKitECM::EventHandlerManipClutch(const prmEventButton & button)
{
    // Pass events
    ClutchEvents.ManipClutch(button);

    // Start manual mode but save the previous state
    if (button.Type() == prmEventButton::PRESSED) {
        ClutchEvents.ManipClutchPreviousState = this->RobotState;
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL);
    } else {
        if (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL) {
            // Enable PID
            PID.Enable(true);
            // set command joint position to joint current
            JointSet.ForceAssign(StateJoint.Position());
            SetPositionJointLocal(JointSet);
            // go back to state before clutching
            SetState(ClutchEvents.ManipClutchPreviousState);
        }
    }
}

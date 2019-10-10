/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
    vctMatRot3 reAlign;
    vct3 axis;
    double angle;
    if (! z.AlmostEqual(shaft, 0.0001)) {
        axis.CrossProductOf(z, shaft);
        angle = acos(vctDotProduct(z, shaft));
        reAlign.From(vctAxAnRot3(axis, angle, VCT_NORMALIZE));
    }

    vctFrm4x4 newGoal;
    newGoal.Translation().Assign(cartesianGoal.Translation());
    newGoal.Rotation().ProductOf(reAlign, cartesianGoal.Rotation());

    if (Manipulator->InverseKinematics(jointSet, newGoal) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = JointsKinematics.Position()[3] - jointSet[3];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[3] = jointSet[3] + differenceInTurns * 2.0 * cmnPI;
        // make sure we are away from RCM point, this test is
        // simplistic
        if (jointSet[2] < 40.0 * cmn_mm) {
            jointSet[2] = 40.0 * cmn_mm;
        }
#if 0
        vctFrm4x4 forward = Manipulator->ForwardKinematics(jointSet);
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

    ToolOffset = 0;

    // set gravity compensation by default
    mGravityCompensation = true;

    // state machine specific to ECM, see base class for other states
    mArmState.AddState("MANUAL");

    // after arm homed
    mArmState.SetTransitionCallback("ARM_HOMED",
                                    &mtsIntuitiveResearchKitECM::TransitionArmHomed,
                                    this);

    mArmState.SetEnterCallback("MANUAL",
                               &mtsIntuitiveResearchKitECM::EnterManual,
                               this);

    mArmState.SetRunCallback("MANUAL",
                             &mtsIntuitiveResearchKitECM::RunManual,
                             this);

    mArmState.SetLeaveCallback("MANUAL",
                               &mtsIntuitiveResearchKitECM::LeaveManual,
                               this);

    // initialize trajectory data
    mJointTrajectory.VelocityMaximum.Assign(30.0 * cmnPI_180, // degrees per second
                                            30.0 * cmnPI_180,
                                            30.0 * cmn_mm,    // mm per second
                                            30.0 * cmnPI_180);
    SetJointVelocityRatio(1.0);
    mJointTrajectory.AccelerationMaximum.Assign(90.0 * cmnPI_180,
                                                90.0 * cmnPI_180,
                                                15.0 * cmn_mm,
                                                90.0 * cmnPI_180);
    SetJointAccelerationRatio(1.0);
    mJointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI / 180.0); // hard coded to 3 degrees

    // default PID tracking errors
    PID.DefaultTrackingErrorTolerance.SetSize(NumberOfJoints());
    PID.DefaultTrackingErrorTolerance.SetAll(7.0 * cmnPI_180); // 7 degrees on angles
    PID.DefaultTrackingErrorTolerance.Element(2) = 10.0 * cmn_mm; // 10 mm

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

void mtsIntuitiveResearchKitECM::ConfigureArmSpecific(const Json::Value & jsonConfig,
                                                      const cmnPath & CMN_UNUSED(configPath),
                                                      const std::string & filename)
{
    // load tool tip transform if any (for up/down endoscopes)
    // future work: add UP/DOWN, _HD and set mass for GC, also create separate method with ROS topic + GUI to set the endoscope
    const Json::Value jsonEndoscope = jsonConfig["endoscope"];
    if (!jsonEndoscope.isNull()) {
        std::string endoscope = jsonEndoscope.asString();
        if (endoscope == "STRAIGHT") {
            ToolOffsetTransformation.Assign( 0.0,  1.0,  0.0,  0.0,
                                            -1.0,  0.0,  0.0,  0.0,
                                             0.0,  0.0,  1.0,  0.0,
                                             0.0,  0.0,  0.0,  1.0);
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": \"endoscope\" type \"" << endoscope
                                     << "\" is not supported.  Options are STRAIGHT (from file \""
                                     << filename << "\")" << std::endl;
            exit(EXIT_FAILURE);
        }
        ToolOffset = new robManipulator(ToolOffsetTransformation);
        Manipulator->Attach(ToolOffset);
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"endoscope\" must be defined (from file \""
                                 << filename << "\")" << std::endl;
        exit(EXIT_FAILURE);
    }

    // check that Rtw0 is not set
    if (Manipulator->Rtw0 != vctFrm4x4::Identity()) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": you can't define the base-offset for the ECM, it is hard coded so gravity compensation works properly.  We always assume the ECM is mounted at 45 degrees! (from file \""
                                 << filename << "\")" << std::endl;
        exit(EXIT_FAILURE);
    }

    // 45 degrees rotation to make sure Z points up, this helps with
    // cisstRobot::robManipulator gravity compensation
    vctFrame4x4<double> Rt(vctMatRot3(1.0,            0.0,            0.0,
                                      0.0,  sqrt(2.0)/2.0,  sqrt(2.0)/2.0,
                                      0.0, -sqrt(2.0)/2.0,  sqrt(2.0)/2.0),
                           vct3(0.0, 0.0, 0.0));
    Manipulator->Rtw0 = Rt;
}

void mtsIntuitiveResearchKitECM::SetGoalHomingArm(void)
{
    // if simulated, start at zero but insert endoscope so it can be used in cartesian mode
    if (mIsSimulated) {
        mJointTrajectory.Goal.SetAll(0.0);
        mJointTrajectory.Goal.at(2) = 12.0 * cmn_cm;
        return;
    }

    // make sure tracking error is set
    PID.SetTrackingErrorTolerance(PID.DefaultTrackingErrorTolerance);
    PID.EnableTrackingError(true);

    // compute joint goal position
    mJointTrajectory.Goal.SetSize(NumberOfJoints());
    if (mHomingGoesToZero) {
        // move to zero position
        mJointTrajectory.Goal.SetAll(0.0);
    } else {
        // stay at current position by default
        mJointTrajectory.Goal.Assign(JointsDesiredPID.Position(), NumberOfJoints());
    }
}

void mtsIntuitiveResearchKitECM::TransitionArmHomed(void)
{
    // on ECM, arm homed means arm ready
    if (mArmState.DesiredStateIsNotCurrent()) {
        mCartesianReady = true;
        mArmState.SetCurrentState("READY");
    }
}

void mtsIntuitiveResearchKitECM::EnterManual(void)
{
    // set ready flag so Arm::GetRobotData updates all joint and
    // cartesian data members
    mJointControlReady = true;
    mCartesianControlReady = true;

    PID.EnableTrackingError(false);
    SetControlEffortActiveJoints();
}

void mtsIntuitiveResearchKitECM::RunManual(void)
{
    // zero efforts
    mEffortJoint.SetAll(0.0);
    if (mGravityCompensation) {
        AddGravityCompensationEfforts(mEffortJoint);
    }
    SetEffortJointLocal(mEffortJoint);
}

void mtsIntuitiveResearchKitECM::LeaveManual(void)
{
    Freeze();
}

void mtsIntuitiveResearchKitECM::EventHandlerTrackingError(void)
{
    RobotInterface->SendError(this->GetName() + ": PID tracking error");
    this->SetDesiredState("UNINITIALIZED");
}

void mtsIntuitiveResearchKitECM::EventHandlerManipClutch(const prmEventButton & button)
{
    // Pass events
    ClutchEvents.ManipClutch(button);

    // Start manual mode but save the previous state
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        if (mArmState.CurrentState() == "READY") {
            ClutchEvents.ManipClutchPreviousState = mArmState.CurrentState();
            mArmState.SetCurrentState("MANUAL");
        } else {
            RobotInterface->SendWarning(this->GetName() + ": arm not ready yet, manipulator clutch ignored");
        }
        break;
    case prmEventButton::RELEASED:
        if (mArmState.CurrentState() == "MANUAL") {
            // go back to state before clutching
            mArmState.SetCurrentState(ClutchEvents.ManipClutchPreviousState);
        }
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitECM::UpdateFeedForward(vctDoubleVec & feedForward)
{
    feedForward.SetAll(0.0);
    AddGravityCompensationEfforts(feedForward);
}

void mtsIntuitiveResearchKitECM::AddGravityCompensationEfforts(vctDoubleVec & efforts)
{
    vctDoubleVec qd(this->NumberOfJointsKinematics(), 0.0);
    efforts.Add(Manipulator->CCG_MDH(JointsKinematics.Position(), qd, 9.81));
}

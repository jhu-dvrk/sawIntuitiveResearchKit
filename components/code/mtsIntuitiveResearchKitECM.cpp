/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

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
        const double difference = JointGet[3] - jointSet[3];
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
    mJointTrajectory.Velocity.Assign(30.0 * cmnPI_180, // degrees per second
                                     30.0 * cmnPI_180,
                                     0.05,            // m per second
                                     20.0 * cmnPI_180);
    mJointTrajectory.Acceleration.Assign(30.0 * cmnPI_180,
                                         30.0 * cmnPI_180,
                                         0.05,
                                         30.0 * cmnPI_180);
    mJointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI / 180.0); // hard coded to 3 degrees
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
            mHomingGoesToZero = jsonHomingGoesToZero.asBool();
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

/*
void mtsIntuitiveResearchKitECM::SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state "
                            << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(newState) << std::endl;

    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED) {
            MessageEvents.Error(this->GetName() + " is not calibrated");
            return;
        }
        // check that the tool is inserted deep enough
        if (JointGet.Element(2) < 40.0 * cmn_mm) {
            MessageEvents.Error(this->GetName() + " can't start cartesian mode, make sure the endoscope is inserted past the cannula (joint 3 > 40 mm)");
        } else {
            if (JointGet.Element(2) < 50.0 * cmn_mm) {
                MessageEvents.Warning(this->GetName() + " cartesian mode started close to RCM (joint 3 < 50 mm), joint 3 will be clamped at 40 mm to avoid moving inside cannula.");
            }
            RobotState = newState;
            if (newState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN) {
                IsGoalSet = false;
                MessageEvents.Status(this->GetName() + " position cartesian");
            } else {
                TrajectoryIsUsed(true);
                MessageEvents.Status(this->GetName() + " position goal cartesian");
            }
        }
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN:
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            MessageEvents.Error(this->GetName() + " is not ready");
            return;
        }
        // check that the tool is inserted deep enough
        if (JointGet.Element(2) < 80.0 * cmn_mm) {
            MessageEvents.Error(this->GetName() + " can't start constraint controller cartesian mode, make sure the tool is inserted past the cannula");
            break;
        }
        RobotState = newState;
        IsGoalSet = false;
        MessageEvents.Status(this->GetName() + " constraint controller cartesian");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL:
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            MessageEvents.Error(this->GetName() + " is not ready yet");
            return;
        }
        // disable PID to allow manual move
        PID.Enable(false);
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " in manual mode");
        break;
    default:
        break;
    }

    // Emit event with current state
    MessageEvents.RobotState(mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(this->RobotState));
}
*/


void mtsIntuitiveResearchKitECM::SetGoalHomingArm(void)
{
    // configure PID to fail in case of tracking error
    vctDoubleVec tolerances(NumberOfJoints());
    tolerances.SetAll(7.0 * cmnPI_180); // 7 degrees on angles
    tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
    PID.SetTrackingErrorTolerance(tolerances);
    PID.EnableTrackingError(true);

    // compute joint goal position
    mJointTrajectory.Goal.SetSize(NumberOfJoints());
    if (mHomingGoesToZero) {
        // move to zero position
        mJointTrajectory.Goal.SetAll(0.0);
    } else {
        // stay at current position by default
        mJointTrajectory.Goal.Assign(JointGetDesired.Goal(), NumberOfJoints());
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

void mtsIntuitiveResearchKitECM::EventHandlerTrackingError(void)
{
    MessageEvents.Error(this->GetName() + ": PID tracking error");
    this->SetDesiredState("UNINITIALIZED");
}

void mtsIntuitiveResearchKitECM::EventHandlerManipClutch(const prmEventButton & button)
{
    // Pass events
    ClutchEvents.ManipClutch(button);

    std::cerr << CMN_LOG_DETAILS << " to be fixed" << std::endl;
    /*
    // Start manual mode but save the previous state
    if (button.Type() == prmEventButton::PRESSED) {
        ClutchEvents.ManipClutchPreviousState = this->RobotState;
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL);
    } else {
        if (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL) {
            // Enable PID
            PID.Enable(true);
            // set command joint position to joint current
            JointSet.ForceAssign(JointGet);
            SetPositionJointLocal(JointSet);
            // go back to state before clutching
            SetState(ClutchEvents.ManipClutchPreviousState);
        }
    }
    */
}

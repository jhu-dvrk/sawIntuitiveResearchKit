/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitPSM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds):
    mtsIntuitiveResearchKitArm(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsIntuitiveResearchKitArm(arg)
{
    Init();
}

robManipulator::Errno mtsIntuitiveResearchKitPSM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal)
{
    if (Manipulator.InverseKinematics(jointSet, cartesianGoal) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = JointGet[3] - jointSet[3];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[3] = jointSet[3] + differenceInTurns * 2.0 * cmnPI;
        // make sure we are away from RCM point, this test is
        // simplistic and might not work with all tools
        if (jointSet[2] < 40.0 * cmn_mm) {
            jointSet[2] = 40.0 * cmn_mm;
        }
        return robManipulator::ESUCCESS;
    }
    return robManipulator::EFAILURE;
}

void mtsIntuitiveResearchKitPSM::Init(void)
{
    // main initialization from base type
    mtsIntuitiveResearchKitArm::Init();

    // initialize trajectory data
    JointTrajectory.Velocity.Ref(2, 0).SetAll(180.0 * cmnPI_180); // degrees per second
    JointTrajectory.Velocity.Element(2) = 0.2; // m per second
    JointTrajectory.Velocity.Ref(4, 3).SetAll(3.0 * 360.0 * cmnPI_180);
    JointTrajectory.Acceleration.Ref(2, 0).SetAll(180.0 * cmnPI_180);
    JointTrajectory.Acceleration.Element(2) = 0.2; // m per second
    JointTrajectory.Acceleration.Ref(4, 3).SetAll(2.0 * 360.0 * cmnPI_180);
    JointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI_180); // hard coded to 3 degrees
    // high values for engage adapter/tool until these use a proper trajectory generator
    PotsToEncodersTolerance.SetAll(15.0 * cmnPI_180); // 15 degrees for rotations
    PotsToEncodersTolerance.Element(2) = 5.0 * cmn_mm; // 5 mm

    // Joint limits when empty
    CouplingChange.NoToolJointLowerLimit.SetSize(NumberOfJoints());
    CouplingChange.NoToolJointUpperLimit.SetSize(NumberOfJoints());
    CouplingChange.NoToolJointLowerLimit.Assign(-91.0, -53.0,   0.0, -175.0, -175.0 , -175.0, -175.0);
    CouplingChange.NoToolJointUpperLimit.Assign( 91.0,  53.0, 240.0,  175.0,  175.0 ,  175.0,  175.0);
    // convert to radians or meters
    CouplingChange.NoToolJointLowerLimit.Ref(2, 0) *= cmnPI_180;
    CouplingChange.NoToolJointLowerLimit.Element(2) *= cmn_mm;
    CouplingChange.NoToolJointLowerLimit.Ref(4, 3) *= cmnPI_180;
    CouplingChange.NoToolJointUpperLimit.Ref(2, 0) *= cmnPI_180;
    CouplingChange.NoToolJointUpperLimit.Element(2) *= cmn_mm;
    CouplingChange.NoToolJointUpperLimit.Ref(4, 3) *= cmnPI_180;

    mtsInterfaceRequired * interfaceRequired;

    // Main interface should have been created by base class init
    CMN_ASSERT(RobotInterface);
    RobotInterface->AddEventWrite(ClutchEvents.ManipClutch, "ManipClutch", prmEventButton());
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetJawPosition, this, "SetJawPosition");

    CMN_ASSERT(IOInterface);
    IOInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::CouplingEventHandler, this, "Coupling");

    CMN_ASSERT(PIDInterface);
    PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EnableJointsEventHandler, this, "EnabledJoints");
    CouplingChange.LastEnabledJoints.SetSize(NumberOfJoints());
    CouplingChange.DesiredEnabledJoints.SetSize(NumberOfJoints());

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

    // Initialize the optimizer
    Optimizer = new mtsIntuitiveResearchKitOptimizer(6);
    Optimizer->InitializeFollowVF(6,
                                  "FollowVFSlave",
                                  "CurrentSlaveKinematics",
                                  "DesiredSlaveKinematics");
}

void mtsIntuitiveResearchKitPSM::Configure(const std::string & filename)
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

        // load DH parameters
        const Json::Value jsonDH = jsonConfig["DH"];
        if (jsonDH.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": can find \"DH\" data in \"" << filename << "\"" << std::endl;
            return;
        }
        this->Manipulator.LoadRobot(jsonDH);
        std::stringstream dhResult;
        this->Manipulator.PrintKinematics(dhResult);
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure " << this->GetName()
                                   << ": loaded kinematrics" << std::endl << dhResult.str() << std::endl;

        // load tool tip transform if any (with warning)
        const Json::Value jsonToolTip = jsonConfig["tooltip-offset"];
        if (jsonToolTip.isNull()) {
            CMN_LOG_CLASS_INIT_WARNING << "Configure " << this->GetName()
                                       << ": can find \"tooltip-offset\" data in \"" << filename << "\"" << std::endl;
        } else {
            cmnDataJSON<vctFrm4x4>::DeSerializeText(ToolOffsetTransformation, jsonToolTip);
            ToolOffset = new robManipulator(ToolOffsetTransformation);
            Manipulator.Attach(ToolOffset);
        }

        // load coupling information (required)
        const Json::Value jsonCoupling = jsonConfig["coupling"];
        if (jsonCoupling.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": can find \"coupling\" data in \"" << filename << "\"" << std::endl;
            return;
        }
        cmnDataJSON<prmActuatorJointCoupling>::DeSerializeText(CouplingChange.ToolCoupling,
                                                               jsonCoupling);

        // load lower/upper position used to engage the tool(required)
        const Json::Value jsonEngagePosition = jsonConfig["tool-engage-position"];
        if (jsonEngagePosition.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": can find \"tool-engage-position\" data in \"" << filename << "\"" << std::endl;
            return;
        }
        // lower
        cmnDataJSON<vctDoubleVec>::DeSerializeText(CouplingChange.ToolEngageLowerPosition,
                                                   jsonEngagePosition["lower"]);
        if (CouplingChange.ToolEngageLowerPosition.size() != NumberOfJoints()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": \"tool-engage-position\" : \"lower\" must contain " << NumberOfJoints()
                                     << " elements in \"" << filename << "\"" << std::endl;
            return;
        }
        // upper
        cmnDataJSON<vctDoubleVec>::DeSerializeText(CouplingChange.ToolEngageUpperPosition,
                                                   jsonEngagePosition["upper"]);
        if (CouplingChange.ToolEngageUpperPosition.size() != NumberOfJoints()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": \"tool-engage-position\" : \"upper\" must contain " << NumberOfJoints()
                                     << " elements in \"" << filename << "\"" << std::endl;
            return;
        }
        // convert to radians or meters
        CouplingChange.ToolEngageUpperPosition.Ref(2, 0) *= cmnPI_180;
        CouplingChange.ToolEngageUpperPosition.Element(2) *= cmn_mm;
        CouplingChange.ToolEngageUpperPosition.Ref(4, 3) *= cmnPI_180;
        CouplingChange.ToolEngageLowerPosition.Ref(2, 0) *= cmnPI_180;
        CouplingChange.ToolEngageLowerPosition.Element(2) *= cmn_mm;
        CouplingChange.ToolEngageLowerPosition.Ref(4, 3) *= cmnPI_180;

        // load lower/upper joint limit for the tool(required)
        const Json::Value jsonJointLimit = jsonConfig["tool-joint-limit"];
        if (jsonJointLimit.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": can find \"tool-joint-limit\" data in \"" << filename << "\"" << std::endl;
            return;
        }
        // lower
        cmnDataJSON<vctDoubleVec>::DeSerializeText(CouplingChange.ToolJointLowerLimit,
                                                   jsonJointLimit["lower"]);
        if (CouplingChange.ToolJointLowerLimit.size() != NumberOfJoints()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": \"tool-joint-limit\" : \"lower\" must contain " << NumberOfJoints()
                                     << " elements in \"" << filename << "\"" << std::endl;
            return;
        }
        // upper
        cmnDataJSON<vctDoubleVec>::DeSerializeText(CouplingChange.ToolJointUpperLimit,
                                                   jsonJointLimit["upper"]);
        if (CouplingChange.ToolJointUpperLimit.size() != NumberOfJoints()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": \"tool-joint-limit\" : \"lower\" must contain " << NumberOfJoints()
                                     << " elements in \"" << filename << "\"" << std::endl;
            return;
        }
        // convert to radians or meters
        CouplingChange.ToolJointUpperLimit.Ref(2, 0) *= cmnPI_180;
        CouplingChange.ToolJointUpperLimit.Element(2) *= cmn_mm;
        CouplingChange.ToolJointUpperLimit.Ref(4, 3) *= cmnPI_180;
        CouplingChange.ToolJointLowerLimit.Ref(2, 0) *= cmnPI_180;
        CouplingChange.ToolJointLowerLimit.Element(2) *= cmn_mm;
        CouplingChange.ToolJointLowerLimit.Ref(4, 3) *= cmnPI_180;

    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is using JSON" << std::endl;
    }
}

void mtsIntuitiveResearchKitPSM::RunArmSpecific(void)
{
    switch (RobotState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_CHANGING_COUPLING:
        RunChangingCoupling();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_ADAPTER:
        RunEngagingAdapter();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_ADAPTER_ENGAGED:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_TOOL:
        RunEngagingTool();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN:
        RunConstraintControllerCartesian();
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitPSM::SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state "
                            << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(newState) << std::endl;

    switch (newState) {

    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        // default coupling, assumes no tool
        {
            const vctDoubleMat identity(vctDoubleMat::Eye(NumberOfAxes()));
            CouplingChange.DesiredCoupling.Assign(identity);
            RobotIO.SetCoupling(CouplingChange.DesiredCoupling);
        }
        // no power
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
        RobotIO.DisablePower();
        PID.Enable(false);
        PID.SetJointLowerLimit(CouplingChange.NoToolJointLowerLimit);
        PID.SetJointUpperLimit(CouplingChange.NoToolJointUpperLimit);
        PID.SetCheckJointLimit(true);
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " not initialized");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER:
        HomingBiasEncoderRequested = false;
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " updating encoders based on potentiometers");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING:
        HomingTimer = 0.0;
        HomingPowerRequested = false;
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " powering");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM:
        HomingCalibrateArmStarted = false;
        RobotState = newState;
        this->MessageEvents.Status(this->GetName() + " calibrating arm");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED:
        RobotState = newState;
        this->MessageEvents.Status(this->GetName() + " arm calibrated");
        // check if adpater is present and trigger new state
        Adapter.GetButton(Adapter.IsPresent);
        if (Adapter.IsPresent) {
            SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_ADAPTER);
        }
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_CHANGING_COUPLING:
        // save previous state
        CouplingChange.PreviousState = RobotState;
        CouplingChange.Started = false;
        // set state
        RobotState = newState;
        this->MessageEvents.Status(this->GetName() + " changing coupling");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_ADAPTER:
        if (RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED) {
            MessageEvents.Status(this->GetName() + " is not calibrated yet, will engage adapter later");
            return;
        }
        // if the tool is present, the adapter is already engadged
        Tool.GetButton(Tool.IsPresent);
        if (Tool.IsPresent) {
            SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ADAPTER_ENGAGED);
            return;
        }
        // start engage procedure
        if (RobotState != mtsIntuitiveResearchKitArmTypes::DVRK_CHANGING_COUPLING) {
            EngagingStage = 0;
        }
        LastEngagingStage = 5;
        // set state
        RobotState = newState;
        this->MessageEvents.Status(this->GetName() + " engaging adapter");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_ADAPTER_ENGAGED:
        RobotState = newState;
        this->MessageEvents.Status(this->GetName() + " adapter engaged");
        // check if tool is present and trigger new state
        Tool.GetButton(Tool.IsPresent);
        if (Tool.IsPresent) {
            SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_TOOL);
        }
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_TOOL:
        if ((RobotState != mtsIntuitiveResearchKitArmTypes::DVRK_CHANGING_COUPLING)
            && (RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_ADAPTER_ENGAGED)) {
            MessageEvents.Status(this->GetName() + " adapter is not engaged yet, will engage tool later");
            return;
        }
        // start engaging procedure
        if (RobotState != mtsIntuitiveResearchKitArmTypes::DVRK_CHANGING_COUPLING) {
            EngagingStage = 0;
        }
        LastEngagingStage = 4;
        // set state
        RobotState = newState;
        this->MessageEvents.Status(this->GetName() + " engaging tool");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_READY:
        {
            PID.SetCheckJointLimit(true);
            vctDoubleVec tolerances(NumberOfJoints());
            // first two rotations
            tolerances.Ref(2, 0).SetAll(20.0 * cmnPI_180);
            // translation
            tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
            // shaft rotation
            tolerances.Element(3) = 120.0 * cmnPI_180;
            // tool orientation
            tolerances.Ref(4, 3).SetAll(35.0 * cmnPI_180);
            // gripper
            tolerances.Element(6) = 90.0 * cmnPI_180; // 90 degrees for gripper, until we change the master gripper matches tool angle
            PID.SetTrackingErrorTolerance(tolerances);
            SetPositionJointLocal(JointGet); // preload PID with current position
            PID.EnableTrackingError(true);
            // set tighter pots/encoder tolerances
            PotsToEncodersTolerance.SetAll(20.0 * cmnPI_180); // 20 degrees for rotations
            PotsToEncodersTolerance.Element(2) = 20.0 * cmn_mm; // 20 mm
            RobotIO.SetPotsToEncodersTolerance(PotsToEncodersTolerance);
        }
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " ready");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT:
        if (RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            MessageEvents.Error(this->GetName() + " is not ready");
            return;
        }
        RobotState = newState;
        JointSet.Assign(JointGetDesired, this->NumberOfJoints());
        if (newState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT) {
            IsGoalSet = false;
            MessageEvents.Status(this->GetName() + " position joint");
        } else {
            JointTrajectory.EndTime = 0.0;
            MessageEvents.Status(this->GetName() + " position goal joint");
        }
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
    {
      if (RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED) {
          MessageEvents.Error(this->GetName() + " is not calibrated");
          return;
      }
      // check that the tool is inserted deep enough
      if (JointGet.Element(2) < 40.0 * cmn_mm) {
          MessageEvents.Error(this->GetName() + " can't start cartesian mode, make sure the tool is inserted past the cannula (joint 3 > 40 mm)");
      } else {
          if (JointGet.Element(2) < 50.0 * cmn_mm) {
              MessageEvents.Warning(this->GetName() + " cartesian mode started close to RCM (joint 3 < 50 mm), joint 3 will be clamped at 40 mm to avoid moving inside cannula.");
          }
          RobotState = newState;
          if (newState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN) {
              IsGoalSet = false;
              MessageEvents.Status(this->GetName() + " position cartesian");
          } else {
              JointTrajectory.EndTime = 0.0;
              MessageEvents.Status(this->GetName() + " position goal cartesian");
          }
      }
      break;
    }
    case mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN:
        if (RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED) {
            MessageEvents.Error(this->GetName() + " is not calibrated");
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
        if (RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED) {
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
    MessageEvents.RobotState(mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(RobotState));
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
        // finally enable PID
        PID.Enable(true);

        // compute joint goal position
        JointTrajectory.Goal.SetSize(NumberOfJoints());
        // check if tool is present
        Tool.GetButton(Tool.IsPresent);
        if (!Tool.IsPresent) {
            // move to zero position only there is no tool present
            JointTrajectory.Goal.SetAll(0.0);
        } else {
            // stay at current position by default
            JointTrajectory.Goal.ForceAssign(JointGet);
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
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED);
        } else {
            // time out
            if (currentTime > HomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingCalibrateArm: unable to reach home position, error in degrees is "
                                           << JointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                MessageEvents.Error(this->GetName() + " unable to reach home position during calibration on pots.");
                this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
            }
        }
    }
}

void mtsIntuitiveResearchKitPSM::RunChangingCoupling(void)
{
    // first phase, disable last 4 joints and wait
    if (!CouplingChange.Started) {
        CouplingChange.DesiredEnabledJoints.Ref(3, 0).SetAll(true);
        CouplingChange.DesiredEnabledJoints.Ref(4, 3).SetAll(false);
        PID.EnableJoints(CouplingChange.DesiredEnabledJoints);
        CouplingChange.WaitingForEnabledJoints = true;
        CouplingChange.ReceivedEnabledJoints = false;
        CouplingChange.Started = true;
        return;
    }

    if (CouplingChange.WaitingForEnabledJoints) {
        // check if received new EnabledJoints
        if (CouplingChange.ReceivedEnabledJoints) {
            if (CouplingChange.DesiredEnabledJoints.Equal(CouplingChange.LastEnabledJoints)) {
                CouplingChange.WaitingForEnabledJoints = false;
                if (CouplingChange.CouplingForTool) {
                    CouplingChange.DesiredCoupling.Assign(CouplingChange.ToolCoupling);
                } else {
                    const vctDoubleMat identity(vctDoubleMat::Eye(NumberOfAxes()));
                    // set desired
                    CouplingChange.DesiredCoupling.Assign(identity);
                }
                RobotIO.SetCoupling(CouplingChange.DesiredCoupling);
                CouplingChange.WaitingForEnabledJoints = false;
                CouplingChange.WaitingForCoupling = true;
                CouplingChange.ReceivedCoupling = false;
                return;
            } else {
                MessageEvents.Warning(this->GetName() + " can't disable last four axis to change coupling.");
                this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED);
            }
        } else {
            return;
        }
    }

    if (CouplingChange.WaitingForCoupling) {
        // check if received new coupling
        if (CouplingChange.ReceivedCoupling) {
            if (CouplingChange.DesiredCoupling.Equal(CouplingChange.LastCoupling)) {
                CouplingChange.WaitingForCoupling = false;
                // now set PID limits based on tool/no tool
                if (CouplingChange.CouplingForTool) {
                    PID.SetJointLowerLimit(CouplingChange.ToolJointLowerLimit);
                    PID.SetJointUpperLimit(CouplingChange.ToolJointUpperLimit);
                } else {
                    PID.SetJointLowerLimit(CouplingChange.NoToolJointLowerLimit);
                    PID.SetJointUpperLimit(CouplingChange.NoToolJointUpperLimit);
                }
                this->SetState(CouplingChange.PreviousState);
            } else {
                MessageEvents.Warning(this->GetName() + " can't set coupling.");
                this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED);
            }
        } else {
            return;
        }
    }
}

void mtsIntuitiveResearchKitPSM::RunEngagingAdapter(void)
{
    const double currentTime = this->StateTable.GetTic();

    if (EngagingStage == 0) {
        // request coupling change
        CouplingChange.PreviousState = RobotState;
        CouplingChange.CouplingForTool = false; // Load identity coupling
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_CHANGING_COUPLING);
        EngagingStage = 1;
        return;
    }

    if (EngagingStage == 1) {
        // configure PID to fail in case of tracking error
        PID.SetCheckJointLimit(false);
        vctDoubleVec tolerances(NumberOfJoints());
        // first two rotations and translation, in case someone is pushinh/holding arm
        tolerances.Ref(2, 0).SetAll(10.0 * cmnPI_180); // 10 degrees
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        // tool/adapter gears should have little resistance?
        tolerances.Ref(4, 3).SetAll(35.0 * cmnPI_180);
        PID.SetTrackingErrorTolerance(tolerances);
        // compute initial time, since we disable power on last 4 use latest read
        vctDoubleVec initialPosition(NumberOfJoints());
        initialPosition.Ref(3, 0).Assign(JointGetDesired.Ref(3, 0));
        initialPosition.Ref(4, 3).Assign(JointGet.Ref(4, 3));
        SetPositionJointLocal(initialPosition);
        // turn on PID
        PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
        PID.EnableTrackingError(true);

        // keep first three joint values as is
        JointTrajectory.Goal.Ref(3, 0).Assign(JointGetDesired.Ref(3, 0));
        // set last 4 to -170.0
        JointTrajectory.Goal.Ref(4, 3).SetAll(-175.0 * cmnPI_180);
        JointTrajectory.LSPB.Set(initialPosition, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime - this->GetPeriodicity(), robLSPB::LSPB_DURATION);
        HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
        EngagingStage = 2;
        return;
    }

    // perform whatever trajectory is going on
    if (currentTime <= HomingTimer) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    } else {
        // check if we were in last phase
        if (EngagingStage > LastEngagingStage) {
            SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ADAPTER_ENGAGED);
        } else {
            if (EngagingStage != LastEngagingStage) {
                JointTrajectory.Goal.Ref(4, 3) *= -1.0; // toggle back and forth
            } else {
                JointTrajectory.Goal.Ref(4, 3).SetAll(0.0); // back to zero position
            }
            JointTrajectory.LSPB.Set(JointGetDesired, JointTrajectory.Goal,
                                     JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                     currentTime - this->GetPeriodicity(), robLSPB::LSPB_DURATION);
            HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
            std::stringstream message;
            message << this->GetName() << " engaging adapter " << EngagingStage - 1 << " of " << LastEngagingStage - 1;
            MessageEvents.Status(message.str());
            EngagingStage++;
        }
    }
}

void mtsIntuitiveResearchKitPSM::RunEngagingTool(void)
{
    const double currentTime = this->StateTable.GetTic();

    if (EngagingStage == 0) {
        // request coupling change
        CouplingChange.PreviousState = RobotState;
        CouplingChange.CouplingForTool = true; // Load tool coupling
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_CHANGING_COUPLING);
        EngagingStage = 1;
        return;
    }

    if (EngagingStage == 1) {
        // configure PID to fail in case of tracking error
        PID.SetCheckJointLimit(false);
        vctDoubleVec tolerances(NumberOfJoints());
        // first two rotations and translation, in case someone is pushinh/holding arm
        tolerances.Ref(2, 0).SetAll(10.0 * cmnPI_180); // 10 degrees
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        // tool/adapter gears should have little resistance?
        tolerances.Ref(4, 3).SetAll(35.0 * cmnPI_180);
        PID.SetTrackingErrorTolerance(tolerances);
        // compute initial time, since we disable power on last 4 use latest read
        vctDoubleVec initialPosition(NumberOfJoints());
        initialPosition.Ref(3, 0).Assign(JointGetDesired.Ref(3, 0));
        initialPosition.Ref(4, 3).Assign(JointGet.Ref(4, 3));
        SetPositionJointLocal(initialPosition);
        // turn on PID
        PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
        PID.EnableTrackingError(true);

        // check if the tool in outside the cannula
        if (JointGet.Element(2) > 50.0 * cmn_mm) {
            std::string message = this->GetName();
            message.append(" tool tip is outside the cannula, assuming it doesn't need to \"engage\".");
            message.append("  If the tool is not engaged properly, move sterile adpater all the way up and re-insert tool.");
            MessageEvents.Status(message);
            SetState(mtsIntuitiveResearchKitArmTypes::DVRK_READY);
        }

        // keep first three joint values as is
        JointTrajectory.Goal.Ref(3, 0).Assign(JointGetDesired.Ref(3, 0));
        // set last 4 to user preferences
        JointTrajectory.Goal.Ref(4, 3).Assign(CouplingChange.ToolEngageLowerPosition.Ref(4, 3));
        JointTrajectory.LSPB.Set(initialPosition, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime - this->GetPeriodicity(), robLSPB::LSPB_DURATION);
        HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
        EngagingStage = 2;
        return;
    }

    // perform whatever trajectory is going on
    if (currentTime <= HomingTimer) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    } else {
        // check if we were in last phase
        if (EngagingStage > LastEngagingStage) {
            SetState(mtsIntuitiveResearchKitArmTypes::DVRK_READY);
        } else {
            if (EngagingStage != LastEngagingStage) {
                // toggle between lower and upper
                if (EngagingStage % 2 == 0) {
                    JointTrajectory.Goal.Ref(4, 3).Assign(CouplingChange.ToolEngageUpperPosition.Ref(4, 3));
                } else {
                    JointTrajectory.Goal.Ref(4, 3).Assign(CouplingChange.ToolEngageLowerPosition.Ref(4, 3));
                }
            } else {
                JointTrajectory.Goal.Ref(4, 3).SetAll(0.0); // back to zero position
            }
            JointTrajectory.LSPB.Set(JointGetDesired, JointTrajectory.Goal,
                                     JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                     currentTime - this->GetPeriodicity(), robLSPB::LSPB_DURATION);
            HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
            std::stringstream message;
            message << this->GetName() << " engaging tool " << EngagingStage - 1 << " of " << LastEngagingStage - 1;
            MessageEvents.Status(message.str());
            EngagingStage++;
        }
    }
}

void mtsIntuitiveResearchKitPSM::RunConstraintControllerCartesian(void)
{
    // Update the optimizer
    // Go through the VF list, update state data pointers, assign tableau references, and fill in the references
    if (IsGoalSet) {
        IsGoalSet = false;

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

void mtsIntuitiveResearchKitPSM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if ((RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN)
        || (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN)) {
        CartesianSetParam = newPosition;
        IsGoalSet = true;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionCartesian: PSM not ready" << std::endl;
    }
}

void mtsIntuitiveResearchKitPSM::SetJawPosition(const double & jawPosition)
{
    const double currentTime = this->StateTable.GetTic();
    switch (RobotState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
    case mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN:
        JointSet[6] = jawPosition;
        IsGoalSet = true;
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
        JointTrajectory.Start.Assign(JointGetDesired, NumberOfJoints());
        // check if there is a trajectory active
        if (currentTime >= JointTrajectory.EndTime) {
            JointTrajectory.Goal.Assign(JointTrajectory.Start);
        }
        JointTrajectory.Goal[6] = jawPosition;
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        JointTrajectory.EndTime = currentTime + JointTrajectory.LSPB.Duration();
        break;
    default:
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetJawPosition: PSM not ready" << std::endl;
        break;
    }
}

void mtsIntuitiveResearchKitPSM::CouplingEventHandler(const prmActuatorJointCoupling & coupling)
{
    CouplingChange.ReceivedCoupling = true;
    CouplingChange.LastCoupling.Assign(coupling);
}

void mtsIntuitiveResearchKitPSM::EnableJointsEventHandler(const vctBoolVec & enable)
{
    CouplingChange.ReceivedEnabledJoints = true;
    CouplingChange.LastEnabledJoints.Assign(enable);
}

void mtsIntuitiveResearchKitPSM::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER);
    } else if ((state == "Cartesian position") || (state == "Teleop")) {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN);
    } else if (state == "Cartesian constraint controller") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN);
    } else if (state == "Manual") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL);
    } else {
        mtsIntuitiveResearchKitArmTypes::RobotStateType stateEnum;
        try {
            stateEnum = mtsIntuitiveResearchKitArmTypes::RobotStateTypeFromString(state);
        } catch (std::exception e) {
            MessageEvents.Error(this->GetName() + ": PSM unsupported state " + state + ": " + e.what());
            return;
        }
        SetState(stateEnum);
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerAdapter(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_ADAPTER);
    } else {
        // this is "down" transition so we have to
        // make sure we had an adapter properly engaged before
        if (RobotState >= mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_ADAPTER) {
            SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED);
        }
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerTool(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ENGAGING_TOOL);
    } else {
        // this is "down" transition so we have to
        // make sure we had a tool properly engaged before
        if (RobotState >= mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            SetState(mtsIntuitiveResearchKitArmTypes::DVRK_ADAPTER_ENGAGED);
        }
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerManipClutch(const prmEventButton & button)
{
    // Pass events
    ClutchEvents.ManipClutch(button);

    // Start manual mode but save the previous state
    if (button.Type() == prmEventButton::PRESSED) {
        ClutchEvents.ManipClutchPreviousState = RobotState;
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL);
    } else {
        if (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL) {
            // set command joint position to joint current
            PID.GetPositionJoint(JointGetParam);
            JointGet.Assign(JointGetParam.Position(), NumberOfJoints());
            JointSet.ForceAssign(JointGet);
            SetPositionJointLocal(JointSet);
            // Enable PID
            PID.Enable(true);
            // go back to state before clutching
            SetState(ClutchEvents.ManipClutchPreviousState);
        }
    }
}

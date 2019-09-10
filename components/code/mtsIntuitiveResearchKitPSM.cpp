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
#include <sawIntuitiveResearchKit/robManipulatorPSMSnake.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
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

void mtsIntuitiveResearchKitPSM::SetSimulated(void)
{
    mtsIntuitiveResearchKitArm::SetSimulated();
    // in simulation mode, we don't need clutch, adapter, tool IO nor Dallas
    RemoveInterfaceRequired("ManipClutch");
    RemoveInterfaceRequired("Adapter");
    RemoveInterfaceRequired("Tool");
    RemoveInterfaceRequired("Dallas");
}

void mtsIntuitiveResearchKitPSM::Init(void)
{
    // main initialization from base type
    mtsIntuitiveResearchKitArm::Init();

    // state machine specific to PSM, see base class for other states
    mArmState.AddState("CHANGING_COUPLING_ADAPTER");
    mArmState.AddState("ENGAGING_ADAPTER");
    mArmState.AddState("ADAPTER_ENGAGED");
    mArmState.AddState("CHANGING_COUPLING_TOOL");
    mArmState.AddState("ENGAGING_TOOL");
    mArmState.AddState("TOOL_ENGAGED");
    mArmState.AddState("MANUAL");

    // after arm homed
    mArmState.SetEnterCallback("ARM_HOMED",
                               &mtsIntuitiveResearchKitPSM::EnterArmHomed,
                               this);
    mArmState.SetRunCallback("ARM_HOMED",
                             &mtsIntuitiveResearchKitPSM::RunArmHomed,
                             this);
    mArmState.SetTransitionCallback("ARM_HOMED",
                                    &mtsIntuitiveResearchKitPSM::TransitionArmHomed,
                                    this);
    mArmState.SetLeaveCallback("ARM_HOMED",
                               &mtsIntuitiveResearchKitPSM::LeaveArmHomed,
                               this);
    mArmState.SetEnterCallback("CHANGING_COUPLING_ADAPTER",
                               &mtsIntuitiveResearchKitPSM::EnterChangingCouplingAdapter,
                               this);
    mArmState.SetRunCallback("CHANGING_COUPLING_ADAPTER",
                             &mtsIntuitiveResearchKitPSM::RunChangingCouplingAdapter,
                             this);
    mArmState.SetEnterCallback("ENGAGING_ADAPTER",
                               &mtsIntuitiveResearchKitPSM::EnterEngagingAdapter,
                               this);
    mArmState.SetRunCallback("ENGAGING_ADAPTER",
                             &mtsIntuitiveResearchKitPSM::RunEngagingAdapter,
                             this);
    mArmState.SetTransitionCallback("ADAPTER_ENGAGED",
                                    &mtsIntuitiveResearchKitPSM::TransitionAdapterEngaged,
                                    this);
    mArmState.SetEnterCallback("CHANGING_COUPLING_TOOL",
                               &mtsIntuitiveResearchKitPSM::EnterChangingCouplingTool,
                               this);
    mArmState.SetRunCallback("CHANGING_COUPLING_TOOL",
                             &mtsIntuitiveResearchKitPSM::RunChangingCouplingAdapter,
                             this);
    mArmState.SetEnterCallback("ENGAGING_TOOL",
                               &mtsIntuitiveResearchKitPSM::EnterEngagingTool,
                               this);
    mArmState.SetRunCallback("ENGAGING_TOOL",
                             &mtsIntuitiveResearchKitPSM::RunEngagingTool,
                             this);
    mArmState.SetTransitionCallback("TOOL_ENGAGED",
                                    &mtsIntuitiveResearchKitPSM::TransitionToolEngaged,
                                    this);
    mArmState.SetEnterCallback("MANUAL",
                               &mtsIntuitiveResearchKitPSM::EnterManual,
                               this);

    // initialize trajectory data
    mJointTrajectory.VelocityMaximum.Ref(2, 0).SetAll(180.0 * cmnPI_180); // degrees per second
    mJointTrajectory.VelocityMaximum.Element(2) = 0.2; // m per second
    mJointTrajectory.VelocityMaximum.Ref(4, 3).SetAll(3.0 * 360.0 * cmnPI_180);
    SetJointVelocityRatio(1.0);
    mJointTrajectory.AccelerationMaximum.Ref(2, 0).SetAll(180.0 * cmnPI_180);
    mJointTrajectory.AccelerationMaximum.Element(2) = 0.2; // m per second
    mJointTrajectory.AccelerationMaximum.Ref(4, 3).SetAll(2.0 * 360.0 * cmnPI_180);
    SetJointAccelerationRatio(1.0);
    mJointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI_180); // hard coded to 3 degrees

    // default PID tracking errors
    PID.DefaultTrackingErrorTolerance.SetSize(NumberOfJoints());
    // first two rotations
    PID.DefaultTrackingErrorTolerance.Ref(2, 0).SetAll(20.0 * cmnPI_180); // 2 elements starting at 0 -> 0 1
    // translation
    PID.DefaultTrackingErrorTolerance.Element(2) = 20.0 * cmn_mm; // 10 mm -> 2
    // shaft rotation and tool orientation
    PID.DefaultTrackingErrorTolerance.Ref(3, 3).SetAll(35.0 * cmnPI_180); // 3 elements starting at 3 0> 3, 4, 5
    // gripper
    PID.DefaultTrackingErrorTolerance.Element(6) = 90.0 * cmnPI_180; // 90 degrees for gripper, until we change the master gripper matches tool angle

    // joint limits when tool is not present
    CouplingChange.NoToolPositionLowerLimit.SetSize(NumberOfJoints());
    CouplingChange.NoToolPositionUpperLimit.SetSize(NumberOfJoints());
    CouplingChange.NoToolPositionLowerLimit.Assign(-91.0 * cmnPI_180,
                                                   -53.0 * cmnPI_180,
                                                   0.0 * cmn_mm,
                                                   -175.0 * cmnPI_180,
                                                   -175.0 * cmnPI_180,
                                                   -175.0 * cmnPI_180,
                                                   -175.0 * cmnPI_180);
    CouplingChange.NoToolPositionUpperLimit.Assign(91.0 * cmnPI_180,
                                                   53.0 * cmnPI_180,
                                                   240.0 * cmn_mm,
                                                   175.0 * cmnPI_180,
                                                   175.0 * cmnPI_180,
                                                   175.0 * cmnPI_180,
                                                   175.0 * cmnPI_180);

    mtsInterfaceRequired * interfaceRequired;

    // Main interface should have been created by base class init
    CMN_ASSERT(RobotInterface);
    Jaw.SetAutomaticTimestamp(false);
    StateTable.AddData(Jaw, "Jaw");

    JawDesired.SetAutomaticTimestamp(false);
    StateTable.AddData(JawDesired, "JawDesired");

    // jaw interface
    RobotInterface->AddCommandReadState(this->StateTable, Jaw, "GetStateJaw");
    RobotInterface->AddCommandReadState(this->StateTable, JawDesired, "GetStateJawDesired");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetPositionJaw, this, "SetPositionJaw");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetPositionGoalJaw, this, "SetPositionGoalJaw");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetEffortJaw, this, "SetEffortJaw");

    // tool specific interface
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetAdapterPresent, this, "SetAdapterPresent");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetToolPresent, this, "SetToolPresent");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetToolType, this, "SetToolType");
    RobotInterface->AddEventWrite(ToolEvents.ToolType, "ToolType", std::string());
    RobotInterface->AddEventVoid(ToolEvents.ToolTypeRequest, "ToolTypeRequest");

    RobotInterface->AddEventWrite(ClutchEvents.ManipClutch, "ManipClutch", prmEventButton());

    CMN_ASSERT(PIDInterface);
    PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::CouplingEventHandler, this, "Coupling");
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

    // Dallas: read tool type from Dallas Chip
    interfaceRequired = AddInterfaceRequired("Dallas");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("TriggerRead", Dallas.TriggerRead);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerToolType, this, "ToolType");
    }
}

void mtsIntuitiveResearchKitPSM::UpdateJointsKinematics(void)
{
    const size_t nbPIDJoints = JointsPID.Name().size();
    const size_t jawIndex = nbPIDJoints - 1;

    if (Jaw.Name().size() == 0) {
        Jaw.Name().SetSize(1);
        Jaw.Name().at(0) = JointsPID.Name().at(jawIndex);
        Jaw.Type().SetSize(1);
        Jaw.Type().at(0) = JointsPID.Type().at(jawIndex);
        Jaw.Position().SetSize(1);
        Jaw.Velocity().SetSize(1);
        Jaw.Effort().SetSize(1);

        JawDesired.Name().SetSize(1);
        JawDesired.Name().at(0) = JointsDesiredPID.Name().at(jawIndex);
        JawDesired.Type().SetSize(1);
        JawDesired.Type().at(0) = JointsDesiredPID.Type().at(jawIndex);
        JawDesired.Position().SetSize(1);
        JawDesired.Velocity().SetSize(0);
        JawDesired.Effort().SetSize(1);
    }

    Jaw.Position().at(0) = JointsPID.Position().at(jawIndex);
    Jaw.Velocity().at(0) = JointsPID.Velocity().at(jawIndex);
    Jaw.Effort().at(0)   = JointsPID.Effort().at(jawIndex);
    Jaw.Timestamp() = JointsPID.Timestamp();
    Jaw.Valid() = JointsPID.Valid();

    JawDesired.Position().at(0) = JointsDesiredPID.Position().at(jawIndex);
    JawDesired.Effort().at(0)   = JointsDesiredPID.Effort().at(jawIndex);
    JawDesired.Timestamp() = JointsDesiredPID.Timestamp();
    JawDesired.Valid() = JointsDesiredPID.Timestamp();

    if (!mSnakeLike) {
        mtsIntuitiveResearchKitArm::UpdateJointsKinematics();
        return;
    }

    if (JointsKinematics.Name().size() != NumberOfJointsKinematics()) {
        JointsKinematics.Name().SetSize(NumberOfJointsKinematics());
        JointsKinematics.Type().SetSize(NumberOfJointsKinematics());
        JointsKinematics.Position().SetSize(NumberOfJointsKinematics());
        JointsKinematics.Velocity().SetSize(NumberOfJointsKinematics());
        JointsKinematics.Effort().SetSize(NumberOfJointsKinematics());

        JointsKinematics.Name().Assign(JointsPID.Name(), 4);
        JointsKinematics.Name().at(4) = JointsPID.Name().at(4) + "1";
        JointsKinematics.Name().at(5) = JointsPID.Name().at(5) + "1";
        JointsKinematics.Name().at(6) = JointsPID.Name().at(5) + "2";
        JointsKinematics.Name().at(7) = JointsPID.Name().at(4) + "2";

        JointsKinematics.Type().Assign(JointsPID.Type(), 4);
        JointsKinematics.Type().at(4) = JointsPID.Type().at(4);
        JointsKinematics.Type().at(5) = JointsPID.Type().at(5);
        JointsKinematics.Type().at(6) = JointsPID.Type().at(5);
        JointsKinematics.Type().at(7) = JointsPID.Type().at(4);
    }

    // Position
    JointsKinematics.Position().Assign(JointsPID.Position(), 4);
    JointsKinematics.Position().at(4) = JointsKinematics.Position().at(7) = JointsPID.Position().at(4) / 2.0;
    JointsKinematics.Position().at(5) = JointsKinematics.Position().at(6) = JointsPID.Position().at(5) / 2.0;

    // Velocity
    JointsKinematics.Velocity().Assign(JointsPID.Velocity(), 4);
    JointsKinematics.Velocity().at(4) = JointsKinematics.Velocity().at(7) = JointsPID.Velocity().at(4) / 2.0;
    JointsKinematics.Velocity().at(5) = JointsKinematics.Velocity().at(6) = JointsPID.Velocity().at(5) / 2.0;

    // Effort
    JointsKinematics.Effort().Assign(JointsPID.Effort(), 4);
    JointsKinematics.Effort().at(4) = JointsKinematics.Effort().at(7) = JointsPID.Effort().at(4) / 2.0;
    JointsKinematics.Effort().at(5) = JointsKinematics.Effort().at(6) = JointsPID.Effort().at(5) / 2.0;
    JointsKinematics.Timestamp() = JointsPID.Timestamp();
    JointsKinematics.Valid() = JointsPID.Valid();

    if (JointsDesiredKinematics.Name().size() != NumberOfJointsKinematics()) {
        JointsDesiredKinematics.Name().SetSize(NumberOfJointsKinematics());
        JointsDesiredKinematics.Type().SetSize(NumberOfJointsKinematics());
        JointsDesiredKinematics.Position().SetSize(NumberOfJointsKinematics());
        JointsDesiredKinematics.Velocity().SetSize(NumberOfJointsKinematics());
        JointsDesiredKinematics.Effort().SetSize(NumberOfJointsKinematics());

        JointsDesiredKinematics.Name().Assign(JointsDesiredPID.Name(), 4);
        JointsDesiredKinematics.Name().at(4) = JointsDesiredPID.Name().at(4) + "1";
        JointsDesiredKinematics.Name().at(5) = JointsDesiredPID.Name().at(5) + "1";
        JointsDesiredKinematics.Name().at(6) = JointsDesiredPID.Name().at(5) + "2";
        JointsDesiredKinematics.Name().at(7) = JointsDesiredPID.Name().at(4) + "2";

        JointsDesiredKinematics.Type().Assign(JointsDesiredPID.Type(), 4);
        JointsDesiredKinematics.Type().at(4) = JointsDesiredPID.Type().at(4);
        JointsDesiredKinematics.Type().at(5) = JointsDesiredPID.Type().at(5);
        JointsDesiredKinematics.Type().at(6) = JointsDesiredPID.Type().at(5);
        JointsDesiredKinematics.Type().at(7) = JointsDesiredPID.Type().at(4);
    }

    // Position
    JointsDesiredKinematics.Position().Assign(JointsDesiredPID.Position(), 4);
    JointsDesiredKinematics.Position().at(4) = JointsDesiredKinematics.Position().at(7) = JointsDesiredPID.Position().at(4) / 2.0;
    JointsDesiredKinematics.Position().at(5) = JointsDesiredKinematics.Position().at(6) = JointsDesiredPID.Position().at(5) / 2.0;

    // Effort
    JointsDesiredKinematics.Effort().Assign(JointsPID.Effort(), 4);
    JointsDesiredKinematics.Effort().at(4) = JointsDesiredKinematics.Effort().at(7) = JointsDesiredPID.Effort().at(4) / 2.0;
    JointsDesiredKinematics.Effort().at(5) = JointsDesiredKinematics.Effort().at(6) = JointsDesiredPID.Effort().at(5) / 2.0;
    JointsDesiredKinematics.Timestamp() = JointsDesiredPID.Timestamp();
    JointsDesiredKinematics.Valid() = JointsDesiredPID.Valid();
}

void mtsIntuitiveResearchKitPSM::ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID)
{
    if (mSnakeLike) {
        CMN_ASSERT(jointsKinematics.size() == 8);
        jointsPID.Assign(jointsKinematics, 4);
        // Test if position 4 and 7 are very much apart; throw error maybe ?
        jointsPID.at(4) = jointsKinematics.at(4) + jointsKinematics.at(7);
        // Same goes for 5 and 6
        jointsPID.at(5) = jointsKinematics.at(5) + jointsKinematics.at(6);
    } else {
        CMN_ASSERT(jointsKinematics.size() >= 6);
        jointsPID.Assign(jointsKinematics, 6);
    }
}

robManipulator::Errno mtsIntuitiveResearchKitPSM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal)
{
    robManipulator::Errno Err;
    if (mSnakeLike) {
        Err = Manipulator->InverseKinematics(jointSet, cartesianGoal);
        // Check for equality Snake joints (4,7) and (5,6)
        if (fabs(jointSet.at(4) - jointSet.at(7)) > 0.00001 ||
                fabs(jointSet.at(5) - jointSet.at(6)) > 0.00001) {
            RobotInterface->SendWarning(GetName() + ": InverseKinematics, equality constraint violated");
        }
    } else {
        Err = Manipulator->InverseKinematics(jointSet, cartesianGoal);
    }

    if (Err == robManipulator::ESUCCESS) {
        // find closest solution mgod 2 pi
        const double difference = JointsKinematics.Position().at(3) - jointSet.at(3);
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet.at(3) = jointSet.at(3) + differenceInTurns * 2.0 * cmnPI;
        // make sure we are away from RCM point, this test is
        // simplistic and might not work with all tools
        if (jointSet.at(2) < 40.0 * cmn_mm) {
            jointSet.at(2) = 40.0 * cmn_mm;
        }
        return robManipulator::ESUCCESS;
    }
    return robManipulator::EFAILURE;
}

void mtsIntuitiveResearchKitPSM::ConfigureArmSpecific(const Json::Value & jsonConfig,
                                                      const cmnPath & configPath,
                                                      const std::string & filename)
{
    // tool detection
    const auto jsonToolDetection = jsonConfig["tool-detection"];
    if (!jsonToolDetection.isNull()) {
        std::string toolDetection = jsonToolDetection.asString();
        mToolDetection = mtsIntuitiveResearchKitToolTypes::DetectionFromString(toolDetection);
        if (mToolDetection == mtsIntuitiveResearchKitToolTypes::FIXED) {
            const auto jsonFixedTool = jsonConfig["tool"];
            if (!jsonFixedTool.isNull()) {
                std::string fixedTool = jsonFixedTool.asString();
                // check if the tool is in the supported list (string name)
                auto found =
                    std::find(mtsIntuitiveResearchKitToolTypes::TypeVectorString().begin(),
                              mtsIntuitiveResearchKitToolTypes::TypeVectorString().end(),
                              fixedTool);
                if (found == mtsIntuitiveResearchKitToolTypes::TypeVectorString().end()) {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmSpecific: " << this->GetName()
                                             << "\"" << fixedTool << "\" found in file \""
                                             << filename << "\" is not a supported type" << std::endl;
                    exit(EXIT_FAILURE);
                } else {
                    mToolType = mtsIntuitiveResearchKitToolTypes::TypeFromString(fixedTool);
                }
                // now look for the file to configure the tool
                cmnPath newConfigPath(configPath);
                newConfigPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share/arm", cmnPath::TAIL);
                std::string fixedToolFile = newConfigPath.Find(fixedTool + ".json");
                if (fixedToolFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmSpecific: " << this->GetName()
                                             << " can't find tool file \""
                                             << fixedTool << ".json\" for tool \""
                                             << fixedTool << "\" defined in file \""
                                             << filename << "\"" << std::endl;
                    exit(EXIT_FAILURE);
                } else {
                    ConfigureTool(fixedToolFile);
                    if (!mToolConfigured) {
                        exit(EXIT_FAILURE);
                    }
                }
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName()
                                         << " can't find field \"tool\" in file \""
                                         << filename << "\" which is required since \"tool-detection\" is set to \"FIXED\"" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    } else {
        mToolDetection = mtsIntuitiveResearchKitToolTypes::AUTOMATIC;
    }
}

void mtsIntuitiveResearchKitPSM::ConfigureTool(const std::string & filename)
{
    std::string fullFilename;
    mToolConfigured = false;

    // try to locate the file based on tool type
    if (cmnPath::Exists(filename)) {
        fullFilename = filename;
    } else {
        // construct path using working directory and share/arm
        cmnPath path(cmnPath::GetWorkingDirectory());
        path.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share/arm", cmnPath::TAIL);
        fullFilename = path.Find(filename);
        // still not found, try to add suffix to search again
        if (fullFilename == "") {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": failed to locate tool file for \""
                                     << filename << "\"" << std::endl;
            return;
        }
    }

    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(fullFilename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": failed to parse configuration file \""
                                     << fullFilename << "\"\n"
                                     << jsonReader.getFormattedErrorMessages();
            return;
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                                   << " using file \"" << fullFilename << "\"" << std::endl
                                   << "----> content of configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

        mSnakeLike = false;
        const Json::Value snakeLike = jsonConfig["snake-like"];
        if (!snakeLike.isNull()) {
            mSnakeLike = snakeLike.asBool();
        }

        // snake require the derived manipulator class so we might
        // have to delete create manipulator

        // preserve Rtw0 just in case we need to create a new instance
        // of robManipulator
        CMN_ASSERT(Manipulator);
        vctFrm4x4 oldRtw0 = Manipulator->Rtw0;
        bool newInstance = false;

        if (mSnakeLike) {
            // maybe we already have it?
            if (!dynamic_cast<robManipulatorPSMSnake *>(this->Manipulator)) {
                delete this->Manipulator;
                this->Manipulator = new robManipulatorPSMSnake();
                newInstance = true;
            }
        } else {
            // make sure we have the base robManipulator class
            if (dynamic_cast<robManipulatorPSMSnake *>(this->Manipulator)) {
                delete this->Manipulator;
                this->Manipulator = new robManipulator();
                newInstance = true;
            }
        }

        // configure new instance and restore Rtw0 in case user have
        // overriden the content of config file
        if (newInstance) {
            ConfigureDH(mConfigurationFile);
            Manipulator->Rtw0.Assign(oldRtw0);
        }

        // remove tool tip offset
        Manipulator->DeleteTools();
        // in any case, we just need the first 3 links
        Manipulator->Truncate(3);

        // now configure the links specific to the tool
        ConfigureDH(jsonConfig, fullFilename);

        // check that the kinematic chain length makes sense
        size_t expectedNumberOfJoint;
        if (mSnakeLike) {
            expectedNumberOfJoint = 8;
        } else {
            expectedNumberOfJoint = 6;
        }
        size_t numberOfJointsLoaded = this->Manipulator->links.size();

        if (expectedNumberOfJoint != numberOfJointsLoaded) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": incorrect number of joints (DH), found "
                                     << numberOfJointsLoaded << ", expected " << expectedNumberOfJoint
                                     << std::endl;
            return;
        }

        // load tool tip transform if any (with warning)
        const Json::Value jsonToolTip = jsonConfig["tooltip-offset"];
        if (jsonToolTip.isNull()) {
            CMN_LOG_CLASS_INIT_WARNING << "ConfigureTool " << this->GetName()
                                       << ": can find \"tooltip-offset\" data in \"" << fullFilename << "\"" << std::endl;
        } else {
            cmnDataJSON<vctFrm4x4>::DeSerializeText(ToolOffsetTransformation, jsonToolTip);
            ToolOffset = new robManipulator(ToolOffsetTransformation);
            Manipulator->Attach(ToolOffset);
        }

        // load coupling information (required)
        const Json::Value jsonCoupling = jsonConfig["coupling"];
        if (jsonCoupling.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"coupling\" data in \"" << fullFilename << "\"" << std::endl;
            return;
        }

        // read 4x4 coupling for last 3 DOFs and jaws
        prmActuatorJointCoupling toolCoupling4;
        cmnDataJSON<prmActuatorJointCoupling>::DeSerializeText(toolCoupling4,
                                                               jsonCoupling);
        // build a coupling matrix for all 7 actuators/dofs
        CouplingChange.ToolCoupling
            .ActuatorToJointPosition().ForceAssign(vctDynamicMatrix<double>::Eye(NumberOfJoints()));
        // assign 4x4 matrix starting at position 3, 3
        CouplingChange.ToolCoupling
            .ActuatorToJointPosition().Ref(4, 4, 3, 3).Assign(toolCoupling4.ActuatorToJointPosition());

        // load jaw data, i.e. joint and torque limits
        const Json::Value jsonJaw = jsonConfig["jaw"];
        if (jsonJaw.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw\" data in \"" << fullFilename << "\"" << std::endl;
            return;
        }
        const Json::Value jsonJawQMin = jsonJaw["qmin"];
        if (jsonJawQMin.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::qmin\" data in \"" << fullFilename << "\"" << std::endl;
            return;
        } else {
            CouplingChange.JawPositionLowerLimit = jsonJawQMin.asDouble();
        }
        const Json::Value jsonJawQMax = jsonJaw["qmax"];
        if (jsonJawQMax.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::qmax\" data in \"" << fullFilename << "\"" << std::endl;
            return;
        } else {
            CouplingChange.JawPositionUpperLimit = jsonJawQMax.asDouble();
        }
        const Json::Value jsonJawFTMax = jsonJaw["ftmax"];
        if (jsonJawFTMax.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::ftmax\" data in \"" << fullFilename << "\"" << std::endl;
            return;
        } else {
            CouplingChange.JawTorqueUpperLimit = jsonJawFTMax.asDouble();
            CouplingChange.JawTorqueLowerLimit = -CouplingChange.JawTorqueUpperLimit;
        }

        // load lower/upper position used to engage the tool(required)
        const Json::Value jsonEngagePosition = jsonConfig["tool-engage-position"];
        if (jsonEngagePosition.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"tool-engage-position\" data in \"" << fullFilename << "\"" << std::endl;
            return;
        }
        // lower
        cmnDataJSON<vctDoubleVec>::DeSerializeText(CouplingChange.ToolEngageLowerPosition,
                                                   jsonEngagePosition["lower"]);
        if (CouplingChange.ToolEngageLowerPosition.size() != 4) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": \"tool-engage-position\" : \"lower\" must contain 4 elements in \""
                                     << fullFilename << "\"" << std::endl;
            return;
        }
        // upper
        cmnDataJSON<vctDoubleVec>::DeSerializeText(CouplingChange.ToolEngageUpperPosition,
                                                   jsonEngagePosition["upper"]);
        if (CouplingChange.ToolEngageUpperPosition.size() != 4) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": \"tool-engage-position\" : \"upper\" must contain 4 elements in \""
                                     << fullFilename << "\"" << std::endl;
            return;
        }

    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName() << ": make sure the file \""
                                 << fullFilename << "\" is in JSON format" << std::endl;
    }

    // update class data member for next steps/states
    mToolConfigured = true;
}

void mtsIntuitiveResearchKitPSM::SetGoalHomingArm(void)
{
    // if simulated, start at zero but insert tool so it can be used in cartesian mode
    if (mIsSimulated) {
        mJointTrajectory.Goal.SetAll(0.0);
        mJointTrajectory.Goal.at(2) = 12.0 * cmn_cm;
        return;
    }

    // check if tool is present and if user wants to go to zero position
    Tool.GetButton(Tool.IsPresent);
    if (mHomingGoesToZero
        && !Tool.IsPresent) {
        // move to zero position only there is no tool present
        mJointTrajectory.Goal.SetAll(0.0);
    } else {
        // stay at current position by default
        mJointTrajectory.Goal.Assign(JointsDesiredPID.Position());
    }
}

void mtsIntuitiveResearchKitPSM::EnterArmHomed(void)
{
    mJointControlReady = true;
}

void mtsIntuitiveResearchKitPSM::RunArmHomed(void)
{
    if (mControlCallback) {
        mControlCallback->Execute();
    }
}

void mtsIntuitiveResearchKitPSM::TransitionArmHomed(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        Adapter.GetButton(Adapter.IsPresent);
        if (Adapter.IsPresent || mIsSimulated || mAdapterNeedEngage) {
            mArmState.SetCurrentState("CHANGING_COUPLING_ADAPTER");
        }
    }
}

void mtsIntuitiveResearchKitPSM::LeaveArmHomed(void)
{
    // turn off joint control ready until we have adapter and tool
    mJointControlReady = false;

    // no control mode defined
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);
}

void mtsIntuitiveResearchKitPSM::RunChangingCoupling(void)
{
    if (mIsSimulated) {
        // now set PID limits based on tool/no tool
        UpdatePIDLimits(CouplingChange.CouplingForTool);
        mArmState.SetCurrentState(CouplingChange.NextState);
        return;
    }

    // first phase, disable last 4 joints and wait
    if (!CouplingChange.Started) {
        // keep first 3 on
        CouplingChange.DesiredEnabledJoints.Ref(3, 0).SetAll(true);
        // turn off last 4
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
                PID.SetCoupling(CouplingChange.DesiredCoupling);
                CouplingChange.WaitingForEnabledJoints = false;
                CouplingChange.WaitingForCoupling = true;
                CouplingChange.ReceivedCoupling = false;
                return;
            } else {
                RobotInterface->SendError(this->GetName() + ": can't disable last four axis to change coupling.");
                mArmState.SetDesiredState(mFallbackState);
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
                UpdatePIDLimits(CouplingChange.CouplingForTool);
                // finally move to next state
                mArmState.SetCurrentState(CouplingChange.NextState);
            } else {
                RobotInterface->SendError(this->GetName() + ": can't set coupling.");
                mArmState.SetDesiredState(mFallbackState);
            }
        } else {
            return;
        }
    }
}

void mtsIntuitiveResearchKitPSM::UpdatePIDLimits(const bool toolPresent)
{
    // now set PID limits based on tool/no tool
    if (toolPresent && mToolConfigured) {
        vctDoubleVec lowerFromKinematics(NumberOfJointsKinematics());
        vctDoubleVec upperFromKinematics(NumberOfJointsKinematics());
        vctDoubleVec lowerToPID(NumberOfJoints());
        vctDoubleVec upperToPID(NumberOfJoints());

        // just to be absolutely totally sure
        CMN_ASSERT(NumberOfJoints() == 7);
        const size_t jawIndex = 6;

        // position limits
        Manipulator->GetJointLimits(lowerFromKinematics,
                                    upperFromKinematics);
        // use kinematic joints... all but last
        lowerToPID.Ref(6).Assign(lowerFromKinematics.Ref(6));
        upperToPID.Ref(6).Assign(upperFromKinematics.Ref(6));
        if (mSnakeLike) {
            // add kinematic joint limits
            lowerToPID(4) += lowerFromKinematics(7);
            lowerToPID(5) += lowerFromKinematics(6);
            upperToPID(4) += upperFromKinematics(7);
            upperToPID(5) += upperFromKinematics(6);
        }
        // ...and jaw
        lowerToPID.at(jawIndex) = CouplingChange.JawPositionLowerLimit;
        upperToPID.at(jawIndex) = CouplingChange.JawPositionUpperLimit;
        // set
        PID.SetPositionLowerLimit(lowerToPID);
        PID.SetPositionUpperLimit(upperToPID);
        // force torque
        Manipulator->GetFTMaximums(upperFromKinematics);
        // use kinematic joints... all but last
        upperToPID.Ref(6).Assign(upperFromKinematics.Ref(6));
        if (mSnakeLike) {
            // add kinematic joint limits
            upperToPID(4) += upperFromKinematics(7);
            upperToPID(5) += upperFromKinematics(6);
        }
        // ...and jaw
        upperToPID.at(jawIndex) = CouplingChange.JawTorqueUpperLimit;
        lowerToPID.ProductOf(-1.0, upperToPID); // manipulator assumes symmetry
        // set
        PID.SetTorqueLowerLimit(lowerToPID);
        PID.SetTorqueUpperLimit(upperToPID);
    } else {
        PID.SetPositionLowerLimit(CouplingChange.NoToolPositionLowerLimit);
        PID.SetPositionUpperLimit(CouplingChange.NoToolPositionUpperLimit);
    }
}

void mtsIntuitiveResearchKitPSM::EnterChangingCouplingAdapter(void)
{
    CouplingChange.Started = false;
    CouplingChange.CouplingForTool = false; // Load identity coupling
    CouplingChange.NextState = "ENGAGING_ADAPTER";
}

void mtsIntuitiveResearchKitPSM::EnterEngagingAdapter(void)
{
    // if simulated, nothing to do
    if (mIsSimulated) {
        return;
    }

    // after coupling is loaded, is it safe to engage?  If a tool is
    // present, the adapter is already engaged
    Tool.GetButton(Tool.IsPresent);
    if (Tool.IsPresent) {
        // we can skip engage later
        mToolNeedEngage = false;
        mArmState.SetCurrentState("ADAPTER_ENGAGED");
        return;
    }
    // if for some reason we don't need to engage, basically, adapter
    // was found before homing
    if (!mAdapterNeedEngage) {
        mArmState.SetCurrentState("ADAPTER_ENGAGED");
        return;
    }

    // other case, initialize variables for adapter engage
    EngagingStage = 1;
    LastEngagingStage = 5;
}

void mtsIntuitiveResearchKitPSM::RunEngagingAdapter(void)
{
    if (mIsSimulated) {
        mArmState.SetCurrentState("ADAPTER_ENGAGED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    if (EngagingStage == 1) {
        // configure PID to fail in case of tracking error
        PID.SetCheckPositionLimit(false);
        vctDoubleVec tolerances(NumberOfJoints());
        // first two rotations and translation, in case someone is pushing/holding arm
        tolerances.Ref(2, 0).SetAll(10.0 * cmnPI_180); // 10 degrees
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        // tool/adapter gears should have little resistance?
        tolerances.Ref(4, 3).SetAll(45.0 * cmnPI_180);
        PID.SetTrackingErrorTolerance(tolerances);
        SetPositionJointLocal(JointsDesiredPID.Position());
        // turn on PID
        PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
        PID.EnableTrackingError(true);

        // make sure we start from current state
        JointSet.Assign(JointsDesiredPID.Position());
        JointVelocitySet.Assign(JointsPID.Velocity());

        // keep first two joint values as is
        mJointTrajectory.Goal.Ref(2, 0).Assign(JointsDesiredPID.Position().Ref(2, 0));
        // sterile adapter should be raised up
        mJointTrajectory.Goal[2] = 0.0;
        // set last 4 to -170.0
        mJointTrajectory.Goal.Ref(4, 3).SetAll(-175.0 * cmnPI_180);
        mJointTrajectory.GoalVelocity.SetAll(0.0);
        mJointTrajectory.EndTime = 0.0;
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
        EngagingStage = 2;
        return;
    }

    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);

    const robReflexxes::ResultType trajectoryResult = mJointTrajectory.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (mJointTrajectory.EndTime == 0.0) {
            mJointTrajectory.EndTime = currentTime + mJointTrajectory.Reflexxes.Duration();
            mHomingTimer = mJointTrajectory.EndTime;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        {
            // check if we were in last phase
            if (EngagingStage > LastEngagingStage) {
                mArmState.SetCurrentState("ADAPTER_ENGAGED");
            } else {
                if (EngagingStage != LastEngagingStage) {
                    mJointTrajectory.Goal.Ref(4, 3) *= -1.0; // toggle back and forth
                } else {
                    mJointTrajectory.Goal.Ref(4, 3).SetAll(0.0); // back to zero position
                }
                mJointTrajectory.EndTime = 0.0;
                std::stringstream message;
                message << this->GetName() << ": engaging adapter " << EngagingStage - 1 << " of " << LastEngagingStage - 1;
                RobotInterface->SendStatus(message.str());
                EngagingStage++;
            }
        }
        break;

    default:
        RobotInterface->SendError(this->GetName() + ": error while evaluating trajectory");
        this->SetDesiredState(mFallbackState);
        break;
    }
}

void mtsIntuitiveResearchKitPSM::TransitionAdapterEngaged(void)
{
    mAdapterNeedEngage = false;
    if (mArmState.DesiredStateIsNotCurrent()) {
        Tool.GetButton(Tool.IsPresent);
        if ((Tool.IsPresent || mIsSimulated) && mToolConfigured) {
            SetToolPresent(true);
            mArmState.SetCurrentState("CHANGING_COUPLING_TOOL");
        }
    }
}

void mtsIntuitiveResearchKitPSM::EnterChangingCouplingTool(void)
{
    CouplingChange.Started = false;
    CouplingChange.CouplingForTool = true; // Load tool coupling
    if (mToolNeedEngage) {
        CouplingChange.NextState = "ENGAGING_TOOL";
    } else {
        CouplingChange.NextState = "TOOL_ENGAGED";
    }
}

void mtsIntuitiveResearchKitPSM::EnterEngagingTool(void)
{
    // set PID limits for tool present
    UpdatePIDLimits(true);

    // if for some reason we don't need to engage, basically, tool was
    // found before homing
    if (!mToolNeedEngage) {
        mArmState.SetCurrentState("TOOL_ENGAGED");
        return;
    }

    // other case, initialize variables for tool engage
    EngagingStage = 1;
    LastEngagingStage = 4;
}

void mtsIntuitiveResearchKitPSM::RunEngagingTool(void)
{
    if (mIsSimulated) {
        mArmState.SetCurrentState("TOOL_ENGAGED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    if (EngagingStage == 1) {
        // configure PID to fail in case of tracking error
        PID.SetCheckPositionLimit(false);
        vctDoubleVec tolerances(NumberOfJoints());
        // first two rotations and translation, in case someone is pushing/holding arm
        tolerances.Ref(2, 0).SetAll(10.0 * cmnPI_180); // 10 degrees
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        // tool/adapter gears should have little resistance?
        tolerances.Ref(4, 3).SetAll(45.0 * cmnPI_180);
        PID.SetTrackingErrorTolerance(tolerances);
        SetPositionJointLocal(JointsDesiredPID.Position());
        // turn on PID
        PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
        PID.EnableTrackingError(true);

        // make sure we start from current state
        JointSet.Assign(JointsDesiredPID.Position());
        JointVelocitySet.Assign(JointsPID.Velocity());

        // check if the tool in outside the cannula
        if (JointsPID.Position().Element(2) > 50.0 * cmn_mm) {
            std::string message = this->GetName();
            message.append(": tool tip is outside the cannula, assuming it doesn't need to \"engage\".");
            message.append("  If the tool is not engaged properly, move the sterile adapter all the way up and re-insert the tool.");
            RobotInterface->SendStatus(message);
            mArmState.SetCurrentState("TOOL_ENGAGED");
        }

        // keep first three joint values as is
        mJointTrajectory.Goal.Ref(3, 0).Assign(JointsDesiredPID.Position().Ref(3, 0));
        // set last 4 to user preferences
        mJointTrajectory.Goal.Ref(4, 3).Assign(CouplingChange.ToolEngageLowerPosition);
        mJointTrajectory.GoalVelocity.SetAll(0.0);
        mJointTrajectory.EndTime = 0.0;
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
        EngagingStage = 2;
        return;
    }

    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);


    const robReflexxes::ResultType trajectoryResult = mJointTrajectory.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (mJointTrajectory.EndTime == 0.0) {
            mJointTrajectory.EndTime = currentTime + mJointTrajectory.Reflexxes.Duration();
            mHomingTimer = mJointTrajectory.EndTime;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        {
            // check if we were in last phase
            if (EngagingStage > LastEngagingStage) {
                mToolNeedEngage = false;
                mArmState.SetCurrentState("TOOL_ENGAGED");
            } else {
                if (EngagingStage != LastEngagingStage) {
                    // toggle between lower and upper
                    if (EngagingStage % 2 == 0) {
                        mJointTrajectory.Goal.Ref(4, 3).Assign(CouplingChange.ToolEngageUpperPosition);
                    } else {
                        mJointTrajectory.Goal.Ref(4, 3).Assign(CouplingChange.ToolEngageLowerPosition);
                    }
                } else {
                    mJointTrajectory.Goal.Ref(4, 3).SetAll(0.0); // back to zero position
                }
                mJointTrajectory.EndTime = 0.0;
                std::stringstream message;
                message << this->GetName() << ": engaging tool " << EngagingStage - 1 << " of " << LastEngagingStage - 1;
                RobotInterface->SendStatus(message.str());
                EngagingStage++;
            }
        }
        break;

    default:
        RobotInterface->SendError(this->GetName() + " error while evaluating trajectory.");
        this->SetDesiredState(mFallbackState);
        break;
    }
}

void mtsIntuitiveResearchKitPSM::TransitionToolEngaged(void)
{
    mToolNeedEngage = false;
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("READY");
    }
}

void mtsIntuitiveResearchKitPSM::EnterManual(void)
{
    PID.Enable(false);
}

void mtsIntuitiveResearchKitPSM::SetPositionJaw(const prmPositionJointSet & jawPosition)
{
    // we need to need to at least ready to control in joint space
    if (!ArmIsReady("SetPositionJaw", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (mControlSpace) {
    case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
        if (! ((mControlMode == mtsIntuitiveResearchKitArmTypes::POSITION_MODE)
               || (mControlMode != mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE))) {
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
            // make sure all other joints have a reasonable cartesian
            // goal for all other joints
            CartesianSetParam.Goal().Assign(CartesianGetDesiredParam.Position());
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
        if (mControlMode != mtsIntuitiveResearchKitArmTypes::POSITION_MODE) {
            // we are initiating the control mode switch so we need to
            // set a reasonable JointSet
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
            // make sure all other joints have a reasonable goal
            JointSet.Assign(JointsDesiredPID.Position(), NumberOfJoints());
        }
        break;
    default:
        // we are initiating the control mode switch
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
        // make sure all other joints have a reasonable goal
        JointSet.Assign(JointsDesiredPID.Position(), NumberOfJoints());
    }

    // save goal
    JawGoal = jawPosition.Goal().at(0);
    mHasNewPIDGoal = true;
}

void mtsIntuitiveResearchKitPSM::SetPositionGoalJaw(const prmPositionJointSet & jawPosition)
{
    // we need to need to at least ready to control in joint space
    if (!ArmIsReady("SetPositionGoalJaw", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (mControlSpace) {
    case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
        if (mControlMode != mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
            // make sure all other joints have a reasonable goal
            mJointTrajectory.Goal.Assign(JointsDesiredPID.Position(), NumberOfJointsKinematics());
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
        if (mControlMode != mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
            // make sure all other joints have a reasonable goal
            mJointTrajectory.Goal.Assign(JointsDesiredPID.Position(), NumberOfJointsKinematics());
        }
        break;
    default:
        // we are initiating the control mode switch
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
        // make sure all other joints have a reasonable goal
        mJointTrajectory.Goal.Assign(JointsDesiredPID.Position());
    }

    // force trajectory re-evaluation with new goal for last joint
    mJointTrajectory.IsWorking = true;
    mJointTrajectory.Goal[6] = jawPosition.Goal().at(0);
    mJointTrajectory.EndTime = 0.0;

    // save position jaw goal, this might lead to jump if the user
    // interupts the jaw trajectory
    JawGoal = jawPosition.Goal().at(0);
}

void mtsIntuitiveResearchKitPSM::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    if (mArmState.CurrentState() != "READY") {
        mtsIntuitiveResearchKitArm::SetPositionJointLocal(newPosition);
        return;
    }

    CMN_ASSERT(JointSetParam.Goal().size() == 7);
    JointSetParam.Goal().Zeros();
    ToJointsPID(newPosition, JointSetParam.Goal());
    JointSetParam.Goal().at(6) = JawGoal;
    JointSetParam.SetTimestamp(StateTable.GetTic());
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitPSM::SetEffortJaw(const prmForceTorqueJointSet & effort)
{
    if (!ArmIsReady("SetEffortJoint", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (mControlSpace) {
    case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
        if (mControlMode != mtsIntuitiveResearchKitArmTypes::EFFORT_MODE) {
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);
            // make sure all other joints have a reasonable cartesian
            // goal
            mWrenchSet.Force().SetAll(0.0);
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
        if (mControlMode != mtsIntuitiveResearchKitArmTypes::EFFORT_MODE) {
            // we are initiating the control mode switch so we need to
            // set a reasonable JointSet
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);
            // make sure all other joints have a reasonable goal
            mEffortJointSet.ForceTorque().SetAll(0.0);
        }
        break;
    default:
        // we are initiating the control mode switch
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);
        // make sure all other joints have a reasonable goal
        mEffortJointSet.ForceTorque().SetAll(0.0);
    }

    // save the desired effort
    EffortJawSet = effort.ForceTorque().at(0);
}

void mtsIntuitiveResearchKitPSM::SetEffortJointLocal(const vctDoubleVec & newEffort)
{
    if (mArmState.CurrentState() != "READY") {
        mtsIntuitiveResearchKitArm::SetEffortJointLocal(newEffort);
        return;
    }

    // pad array for PID
    vctDoubleVec torqueDesired(NumberOfJoints(), 0.0); // for PID
    if (mSnakeLike) {
        std::cerr << CMN_LOG_DETAILS << " need to convert 8 joints from snake to 6 for PID control" << std::endl;
    } else {
        torqueDesired.Assign(mEffortJoint, NumberOfJointsKinematics());
    }
    torqueDesired.at(6) = EffortJawSet;

    // convert to cisstParameterTypes
    mTorqueSetParam.SetForceTorque(torqueDesired);
    mTorqueSetParam.SetTimestamp(StateTable.GetTic());
    PID.SetTorqueJoint(mTorqueSetParam);
}

void mtsIntuitiveResearchKitPSM::CouplingEventHandler(const prmActuatorJointCoupling & coupling)
{
    CouplingChange.ReceivedCoupling = true;
    CouplingChange.LastCoupling.Assign(coupling);
    // refresh robot data
    GetRobotData();
}

void mtsIntuitiveResearchKitPSM::EnableJointsEventHandler(const vctBoolVec & enable)
{
    CouplingChange.ReceivedEnabledJoints = true;
    CouplingChange.LastEnabledJoints.Assign(enable);
}

void mtsIntuitiveResearchKitPSM::SetAdapterPresent(const bool & present)
{
    if (present) {
        // we will need to engage this adapter
        mAdapterNeedEngage = true;
    } else {
        mCartesianReady = false;
        mArmState.SetCurrentState("ARM_HOMED");
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerAdapter(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        SetAdapterPresent(true);
        break;
    case prmEventButton::RELEASED:
        SetAdapterPresent(false);
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitPSM::SetToolPresent(const bool & present)
{
    if (present) {
        // we will need to engage this tool
        mToolNeedEngage = true;
        ToolEvents.ToolType(mtsIntuitiveResearchKitToolTypes::TypeToString(mToolType));
    } else {
        mCartesianReady = false;
        mArmState.SetCurrentState("ADAPTER_ENGAGED");
        ToolEvents.ToolType(std::string());
    }
}

void mtsIntuitiveResearchKitPSM::SetToolType(const std::string & toolType)
{
    if (mToolTypeRequested) {
        EventHandlerToolType(toolType);
        if (mToolConfigured) {
            mToolTypeRequested = false;
        }
    } else {
        RobotInterface->SendWarning(this->GetName() + ": received request to set tool type but not expecting it now.  Request ignored.");
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerTool(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        switch (mToolDetection) {
        case mtsIntuitiveResearchKitToolTypes::AUTOMATIC:
            Dallas.TriggerRead();
            break;
        case mtsIntuitiveResearchKitToolTypes::MANUAL:
            mToolTypeRequested = true;
            ToolEvents.ToolTypeRequest();
            break;
        case mtsIntuitiveResearchKitToolTypes::FIXED:
            SetToolPresent(true);
            break;
        default:
            break;
        }
        break;
    case prmEventButton::RELEASED:
        switch (mToolDetection) {
        case mtsIntuitiveResearchKitToolTypes::AUTOMATIC:
        case mtsIntuitiveResearchKitToolTypes::MANUAL:
            mToolConfigured = false;
            SetToolPresent(false);
            break;
        case mtsIntuitiveResearchKitToolTypes::FIXED:
            SetToolPresent(false);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerManipClutch(const prmEventButton & button)
{
    // Pass events
    ClutchEvents.ManipClutch(button);

    // Start manual mode but save the previous state
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        ClutchEvents.ManipClutchPreviousState = mArmState.CurrentState();
        PID.Enabled(ClutchEvents.PIDEnabledPreviousState);
        mArmState.SetCurrentState("MANUAL");
        break;
    case prmEventButton::RELEASED:
        if (mArmState.CurrentState() == "MANUAL") {
            // go back to state before clutching
            mArmState.SetCurrentState(ClutchEvents.ManipClutchPreviousState);
            PID.Enable(ClutchEvents.PIDEnabledPreviousState);
        }
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerToolType(const std::string & toolType)
{
    RobotInterface->SendStatus(this->GetName() + ": setting up for tool type \"" + toolType + "\"");
    // check if the tool is in the supported list
    auto found =
        std::find(mtsIntuitiveResearchKitToolTypes::TypeVectorString().begin(),
                  mtsIntuitiveResearchKitToolTypes::TypeVectorString().end(),
                  toolType);
    if (found == mtsIntuitiveResearchKitToolTypes::TypeVectorString().end()) {
        RobotInterface->SendError(this->GetName() + ": tool type \"" + toolType + "\" is not supported");
        ToolEvents.ToolType(std::string("ERROR"));
        return;
    } else {
        mToolType = mtsIntuitiveResearchKitToolTypes::TypeFromString(toolType);
    }
    // supported tools
    ConfigureTool(toolType + ".json");
    if (mToolConfigured) {
        SetToolPresent(true);
    } else {
        RobotInterface->SendError(this->GetName() + ": failed to configure tool \"" + toolType + "\", check terminal output and cisstLog file");
        ToolEvents.ToolType(std::string("ERROR"));
    }
}

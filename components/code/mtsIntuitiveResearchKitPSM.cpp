/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <cisstCommon/cmnPath.h>
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

void mtsIntuitiveResearchKitPSM::PostConfigure(const Json::Value & jsonConfig,
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
                    CMN_LOG_CLASS_INIT_ERROR << "PostConfigure: " << this->GetName()
                                             << "\"" << fixedTool << "\" found in file \""
                                             << filename << "\" is not a supported type" << std::endl;
                    exit(EXIT_FAILURE);
                } else {
                    mToolType = mtsIntuitiveResearchKitToolTypes::TypeFromString(fixedTool);
                }
                // now look for the file to configure the tool
                std::string fixedToolFile = configPath.Find(fixedTool + ".json");
                if (fixedToolFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "PostConfigure: " << this->GetName()
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
                CMN_LOG_CLASS_INIT_ERROR << "PostConfigure: " << this->GetName()
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
        path.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share/tool", cmnPath::TAIL);
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

        // keep info in log
        std::stringstream dhResult;
        this->Manipulator->PrintKinematics(dhResult);
        CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureTool " << this->GetName()
                                   << ": loaded kinematics" << std::endl << dhResult.str() << std::endl;

        // update ConfigurationJointKinematic from manipulator
        UpdateConfigurationJointKinematic();

        // resize data members using kinematics (jacobians and effort vectors)
        ResizeKinematicsData();

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
            CouplingChange.jaw_configuration_js.PositionMin().SetSize(1);
            CouplingChange.jaw_configuration_js.PositionMin().at(0) = jsonJawQMin.asDouble();
        }
        const Json::Value jsonJawQMax = jsonJaw["qmax"];
        if (jsonJawQMax.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::qmax\" data in \"" << fullFilename << "\"" << std::endl;
            return;
        } else {
            CouplingChange.jaw_configuration_js.PositionMax().SetSize(1);
            CouplingChange.jaw_configuration_js.PositionMax().at(0) = jsonJawQMax.asDouble();
        }
        const Json::Value jsonJawFTMax = jsonJaw["ftmax"];
        if (jsonJawFTMax.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::ftmax\" data in \"" << fullFilename << "\"" << std::endl;
            return;
        } else {
            CouplingChange.jaw_configuration_js.EffortMin().SetSize(1);
            CouplingChange.jaw_configuration_js.EffortMax().SetSize(1);
            CouplingChange.jaw_configuration_js.EffortMax().at(0) = jsonJawFTMax.asDouble();
            CouplingChange.jaw_configuration_js.EffortMin().at(0) = -jsonJawFTMax.asDouble();
        }

        CouplingChange.jaw_configuration_js.Name().SetSize(1);
        CouplingChange.jaw_configuration_js.Name().at(0) = "jaw";
        CouplingChange.jaw_configuration_js.Type().SetSize(1);
        CouplingChange.jaw_configuration_js.Type().at(0) = PRM_JOINT_REVOLUTE;

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

void mtsIntuitiveResearchKitPSM::UpdateStateJointKinematics(void)
{
    const size_t nbPIDJoints = m_measured_js_pid.Name().size();
    const size_t jawIndex = nbPIDJoints - 1;

    if (m_jaw_measured_js.Name().size() == 0) {
        m_jaw_measured_js.Name().SetSize(1);
        m_jaw_measured_js.Name().at(0) = m_measured_js_pid.Name().at(jawIndex);
        m_jaw_measured_js.Position().SetSize(1);
        m_jaw_measured_js.Velocity().SetSize(1);
        m_jaw_measured_js.Effort().SetSize(1);

        m_jaw_setpoint_js.Name().SetSize(1);
        m_jaw_setpoint_js.Name().at(0) = m_setpoint_js_pid.Name().at(jawIndex);
        m_jaw_setpoint_js.Position().SetSize(1);
        m_jaw_setpoint_js.Velocity().SetSize(0);
        m_jaw_setpoint_js.Effort().SetSize(1);
    }

    m_jaw_measured_js.Position().at(0) = m_measured_js_pid.Position().at(jawIndex);
    m_jaw_measured_js.Velocity().at(0) = m_measured_js_pid.Velocity().at(jawIndex);
    m_jaw_measured_js.Effort().at(0)   = m_measured_js_pid.Effort().at(jawIndex);
    m_jaw_measured_js.Timestamp() = m_measured_js_pid.Timestamp();
    m_jaw_measured_js.Valid() = m_measured_js_pid.Valid();

    m_jaw_setpoint_js.Position().at(0) = m_setpoint_js_pid.Position().at(jawIndex);
    m_jaw_setpoint_js.Effort().at(0)   = m_setpoint_js_pid.Effort().at(jawIndex);
    m_jaw_setpoint_js.Timestamp() = m_setpoint_js_pid.Timestamp();
    m_jaw_setpoint_js.Valid() = m_setpoint_js_pid.Timestamp();

    if (!mSnakeLike) {
        mtsIntuitiveResearchKitArm::UpdateStateJointKinematics();
        return;
    }

    // Position
    m_measured_js_kin.Position().Assign(m_measured_js_pid.Position(), 4);
    m_measured_js_kin.Position().at(4) = m_measured_js_kin.Position().at(7) = m_measured_js_pid.Position().at(4) / 2.0;
    m_measured_js_kin.Position().at(5) = m_measured_js_kin.Position().at(6) = m_measured_js_pid.Position().at(5) / 2.0;

    // Velocity
    m_measured_js_kin.Velocity().Assign(m_measured_js_pid.Velocity(), 4);
    m_measured_js_kin.Velocity().at(4) = m_measured_js_kin.Velocity().at(7) = m_measured_js_pid.Velocity().at(4) / 2.0;
    m_measured_js_kin.Velocity().at(5) = m_measured_js_kin.Velocity().at(6) = m_measured_js_pid.Velocity().at(5) / 2.0;

    // Effort
    m_measured_js_kin.Effort().Assign(m_measured_js_pid.Effort(), 4);
    m_measured_js_kin.Effort().at(4) = m_measured_js_kin.Effort().at(7) = m_measured_js_pid.Effort().at(4) / 2.0;
    m_measured_js_kin.Effort().at(5) = m_measured_js_kin.Effort().at(6) = m_measured_js_pid.Effort().at(5) / 2.0;
    m_measured_js_kin.Timestamp() = m_measured_js_pid.Timestamp();
    m_measured_js_kin.Valid() = m_measured_js_pid.Valid();

    // Position
    m_setpoint_js_kin.Position().Assign(m_setpoint_js_pid.Position(), 4);
    m_setpoint_js_kin.Position().at(4) = m_setpoint_js_kin.Position().at(7) = m_setpoint_js_pid.Position().at(4) / 2.0;
    m_setpoint_js_kin.Position().at(5) = m_setpoint_js_kin.Position().at(6) = m_setpoint_js_pid.Position().at(5) / 2.0;

    // Effort
    m_setpoint_js_kin.Effort().Assign(m_measured_js_pid.Effort(), 4);
    m_setpoint_js_kin.Effort().at(4) = m_setpoint_js_kin.Effort().at(7) = m_setpoint_js_pid.Effort().at(4) / 2.0;
    m_setpoint_js_kin.Effort().at(5) = m_setpoint_js_kin.Effort().at(6) = m_setpoint_js_pid.Effort().at(5) / 2.0;
    m_setpoint_js_kin.Timestamp() = m_setpoint_js_pid.Timestamp();
    m_setpoint_js_kin.Valid() = m_setpoint_js_pid.Valid();
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
        const double difference = m_measured_js_kin.Position().at(3) - jointSet.at(3);
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet.at(3) = jointSet.at(3) + differenceInTurns * 2.0 * cmnPI;
        // make sure we are away from RCM point, this test is
        // simplistic and might not work with all tools
        if (jointSet.at(2) < mtsIntuitiveResearchKit::PSMOutsideCannula) {
            jointSet.at(2) = mtsIntuitiveResearchKit::PSMOutsideCannula;
        }
        return robManipulator::ESUCCESS;
    }
    return robManipulator::EFAILURE;
}

void mtsIntuitiveResearchKitPSM::Init(void)
{
    // main initialization from base type
    mtsIntuitiveResearchKitArm::Init();

    // state machine specific to PSM, see base class for other states
    mArmState.AddState("CHANGING_COUPLING_ADAPTER");
    mArmState.AddState("ENGAGING_ADAPTER");
    mArmState.AddState("CHANGING_COUPLING_TOOL");
    mArmState.AddState("ENGAGING_TOOL");
    mArmState.AddState("TOOL_ENGAGED");
    mArmState.AddState("MANUAL");

    // after arm homed
    mArmState.SetTransitionCallback("HOMED",
                                    &mtsIntuitiveResearchKitPSM::TransitionHomed,
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
    mArmState.SetEnterCallback("TOOL_ENGAGED",
                               &mtsIntuitiveResearchKitPSM::EnterToolEngaged,
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
    CouplingChange.NoToolConfiguration.Name().SetSize(7);
    CouplingChange.NoToolConfiguration.Name().at(0) = "outer_yaw";
    CouplingChange.NoToolConfiguration.Name().at(1) = "outer_pitch";
    CouplingChange.NoToolConfiguration.Name().at(2) = "outer_insertion";
    CouplingChange.NoToolConfiguration.Name().at(3) = "disc_1";
    CouplingChange.NoToolConfiguration.Name().at(4) = "disc_2";
    CouplingChange.NoToolConfiguration.Name().at(5) = "disc_3";
    CouplingChange.NoToolConfiguration.Name().at(6) = "disc_4";
    CouplingChange.NoToolConfiguration.Type().SetSize(7);
    CouplingChange.NoToolConfiguration.Type().SetAll(PRM_JOINT_REVOLUTE);
    CouplingChange.NoToolConfiguration.PositionMin().SetSize(NumberOfJoints());
    CouplingChange.NoToolConfiguration.PositionMax().SetSize(NumberOfJoints());
    CouplingChange.NoToolConfiguration.PositionMin().Assign(-91.0 * cmnPI_180,
                                                            -53.0 * cmnPI_180,
                                                            0.0 * cmn_mm,
                                                            -175.0 * cmnPI_180,
                                                            -175.0 * cmnPI_180,
                                                            -175.0 * cmnPI_180,
                                                            -175.0 * cmnPI_180);
    CouplingChange.NoToolConfiguration.PositionMax().Assign(91.0 * cmnPI_180,
                                                            53.0 * cmnPI_180,
                                                            240.0 * cmn_mm,
                                                            175.0 * cmnPI_180,
                                                            175.0 * cmnPI_180,
                                                            175.0 * cmnPI_180,
                                                            175.0 * cmnPI_180);
    // using values from sawControllersPID.xml: max_amp / nm_to_amp
    CouplingChange.NoToolConfiguration.EffortMin().SetSize(NumberOfJoints());
    CouplingChange.NoToolConfiguration.EffortMax().SetSize(NumberOfJoints());
    CouplingChange.NoToolConfiguration.EffortMax().Assign(3.316101,
                                                          3.316101,
                                                          9.877926,
                                                          0.343642,
                                                          0.343642,
                                                          0.343642,
                                                          0.343642);
    CouplingChange.NoToolConfiguration.EffortMin().Assign(-CouplingChange.NoToolConfiguration.EffortMax());

    mtsInterfaceRequired * interfaceRequired;

    // Main interface should have been created by base class init
    CMN_ASSERT(RobotInterface);
    m_jaw_measured_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_jaw_measured_js, "m_jaw_measured_js");

    m_jaw_setpoint_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_jaw_setpoint_js, "m_jaw_setpoint_js");

    // state table for configuration
    mStateTableConfiguration.AddData(CouplingChange.jaw_configuration_js, "jaw_configuration_js");

    // jaw interface
    RobotInterface->AddCommandReadState(this->StateTable, m_jaw_measured_js, "jaw_measured_js");
    RobotInterface->AddCommandReadState(this->StateTable, m_jaw_setpoint_js, "jaw_setpoint_js");
    RobotInterface->AddCommandReadState(this->mStateTableConfiguration,
                                        CouplingChange.jaw_configuration_js, "jaw_configuration_js");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_servo_jp, this, "jaw_servo_jp");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_move_jp, this, "jaw_move_jp");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_servo_jf, this, "jaw_servo_jf");

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

bool mtsIntuitiveResearchKitPSM::IsHomed(void) const
{
    if (Tool.IsPresent) {
        return m_powered && m_encoders_biased_from_pots && Adapter.IsEngaged && Tool.IsEngaged;
    }
    if (Adapter.IsPresent) {
        return m_powered && m_encoders_biased_from_pots && Adapter.IsEngaged;
    }
    return m_powered && m_encoders_biased_from_pots;
}

void mtsIntuitiveResearchKitPSM::UnHome(void)
{
    if (Tool.IsPresent) {
        Tool.IsEngaged = false;
        return;
    }
    if (Adapter.IsPresent) {
        Adapter.IsEngaged = false;
        return;
    }
    // to force re-bias on pots
    m_re_home = true;
    m_encoders_biased_from_pots = false;
}

bool mtsIntuitiveResearchKitPSM::IsJointReady(void) const
{
    return m_powered && m_encoders_biased_from_pots;
}

bool mtsIntuitiveResearchKitPSM::IsCartesianReady(void) const
{
    return m_powered && m_encoders_biased_from_pots && Tool.IsEngaged;
}

void mtsIntuitiveResearchKitPSM::SetGoalHomingArm(void)
{
    // if simulated, start at zero but insert tool so it can be used in cartesian mode
    if (m_simulated) {
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
        mJointTrajectory.Goal.Assign(m_setpoint_js_pid.Position());
    }
}

void mtsIntuitiveResearchKitPSM::TransitionHomed(void)
{
    Adapter.GetButton(Adapter.IsPresent);
    if (!Adapter.IsPresent) {
        Adapter.IsEngaged = false;
    }
    if (!Adapter.IsEngaged &&
        (Adapter.IsPresent || m_simulated || Adapter.NeedEngage)) {
        mArmState.SetCurrentState("CHANGING_COUPLING_ADAPTER");
    }

    Tool.GetButton(Tool.IsPresent);
    if (!Tool.IsPresent) {
        Tool.IsEngaged = false;
    }
    if (Adapter.IsEngaged && !Tool.IsEngaged) {
        if (!m_simulated) {
            if (Tool.IsPresent && mToolConfigured) {
                SetToolPresent(true);
                mArmState.SetCurrentState("CHANGING_COUPLING_TOOL");
            }
        } else {
            // simulated case
            SetToolPresent(true);
            // check if tool is configured, i.e. fixed or manual
            if (mToolConfigured) {
                mArmState.SetCurrentState("CHANGING_COUPLING_TOOL");
            } else {
                // request tool type if needed
                if (!mToolTypeRequested) {
                    mToolTypeRequested = true;
                    ToolEvents.ToolTypeRequest();
                    RobotInterface->SendWarning(this->GetName() + ": tool type requested from user");
                }
            }
        }
    }
}

void mtsIntuitiveResearchKitPSM::RunChangingCoupling(void)
{
    if (m_simulated) {
        // now set PID limits based on tool/no tool
        UpdateConfigurationJointPID(CouplingChange.CouplingForTool);
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
                    const vctDoubleMat identity(vctDoubleMat::Eye(NumberOfJoints()));
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
                UpdateConfigurationJointPID(CouplingChange.CouplingForTool);
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

void mtsIntuitiveResearchKitPSM::UpdateConfigurationJointPID(const bool toolPresent)
{
    // now set PID limits based on tool/no tool
    if (toolPresent && mToolConfigured) {

        // get names, types and joint limits for kinematics config from the manipulator
        // name and types need conversion
        mStateTableConfiguration.Start();
        m_configuration_js_pid.Name().SetSize(NumberOfJoints());
        m_configuration_js_pid.Type().SetSize(NumberOfJoints());
        std::vector<std::string> names(NumberOfJointsKinematics());
        std::vector<robJoint::Type> types(NumberOfJointsKinematics());
        this->Manipulator->GetJointNames(names);
        this->Manipulator->GetJointTypes(types);
        for (size_t index = 0; index < NumberOfJointsKinematics(); ++index) {
            m_configuration_js_pid.Name().at(index) = names.at(index);
            switch (types.at(index)) {
            case robJoint::HINGE:
                m_configuration_js_pid.Type().at(index) = PRM_JOINT_REVOLUTE;
                break;
            case robJoint::SLIDER:
                m_configuration_js_pid.Type().at(index) = PRM_JOINT_PRISMATIC;
                break;
            default:
                m_configuration_js_pid.Type().at(index) = PRM_JOINT_UNDEFINED;
                break;
            }
        }

        // limits need to take into account snake case
        vctDoubleVec lowerFromKinematics(NumberOfJointsKinematics());
        vctDoubleVec upperFromKinematics(NumberOfJointsKinematics());

        m_configuration_js_pid.PositionMin().SetSize(NumberOfJoints());
        m_configuration_js_pid.PositionMax().SetSize(NumberOfJoints());
        m_configuration_js_pid.EffortMin().SetSize(NumberOfJoints());
        m_configuration_js_pid.EffortMax().SetSize(NumberOfJoints());

        // just to be absolutely totally sure
        CMN_ASSERT(NumberOfJoints() == 7);
        const size_t jawIndex = 6;

        // position limits
        Manipulator->GetJointLimits(lowerFromKinematics,
                                    upperFromKinematics);
        // use kinematic joints... all but last
        m_configuration_js_pid.PositionMin().Ref(6).Assign(lowerFromKinematics.Ref(6));
        m_configuration_js_pid.PositionMax().Ref(6).Assign(upperFromKinematics.Ref(6));
        if (mSnakeLike) {
            // add kinematic joint limits
            m_configuration_js_pid.PositionMin().at(4) += lowerFromKinematics.at(7);
            m_configuration_js_pid.PositionMin().at(5) += lowerFromKinematics.at(6);
            m_configuration_js_pid.PositionMax().at(4) += upperFromKinematics.at(7);
            m_configuration_js_pid.PositionMax().at(5) += upperFromKinematics.at(6);
        }
        // ...and jaw
        m_configuration_js_pid.PositionMin().at(jawIndex) = CouplingChange.jaw_configuration_js.PositionMin().at(0);
        m_configuration_js_pid.PositionMax().at(jawIndex) = CouplingChange.jaw_configuration_js.PositionMax().at(0);

        // force torque
        Manipulator->GetFTMaximums(upperFromKinematics);
        // use kinematic joints... all but last
        m_configuration_js_pid.EffortMax().Ref(6).Assign(upperFromKinematics.Ref(6));
        if (mSnakeLike) {
            // add kinematic joint limits
            m_configuration_js_pid.EffortMax().at(4) += upperFromKinematics.at(7);
            m_configuration_js_pid.EffortMax().at(5) += upperFromKinematics.at(6);
        }
        // ...and jaw
        m_configuration_js_pid.EffortMax().at(jawIndex) = CouplingChange.jaw_configuration_js.EffortMax().at(0);
        m_configuration_js_pid.EffortMin().ProductOf(-1.0, m_configuration_js_pid.EffortMax()); // manipulator assumes symmetry

        mStateTableConfiguration.Advance();

        // set
        PID.configure_js(m_configuration_js_pid);
    } else {
        PID.configure_js(CouplingChange.NoToolConfiguration);
    }
}

void mtsIntuitiveResearchKitPSM::EnterChangingCouplingAdapter(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    CouplingChange.Started = false;
    CouplingChange.CouplingForTool = false; // Load identity coupling
    CouplingChange.NextState = "ENGAGING_ADAPTER";
}

void mtsIntuitiveResearchKitPSM::EnterEngagingAdapter(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    // if simulated, nothing to do
    if (m_simulated) {
        return;
    }

    // after coupling is loaded, is it safe to engage?  If a tool is
    // present, the adapter is already engaged
    Tool.GetButton(Tool.IsPresent);
    if (Tool.IsPresent) {
        // we can skip engage later
        Tool.NeedEngage = false;
        Adapter.IsEngaged = true;
        mArmState.SetCurrentState("HOMED");
        return;
    }
    // if for some reason we don't need to engage, basically, adapter
    // was found before homing
    if (!Adapter.NeedEngage) {
        Adapter.IsEngaged = true;
        mArmState.SetCurrentState("HOMED");
        return;
    }

    // other case, initialize variables for adapter engage
    EngagingStage = 1;
    LastEngagingStage = 5;
}

void mtsIntuitiveResearchKitPSM::RunEngagingAdapter(void)
{
    if (m_simulated) {
        Adapter.NeedEngage = false;
        Adapter.IsEngaged = true;
        mArmState.SetCurrentState("HOMED");
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
        SetPositionJointLocal(m_setpoint_js_pid.Position());
        // turn on PID
        PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
        PID.EnableTrackingError(true);

        // make sure we start from current state
        JointSet.Assign(m_setpoint_js_pid.Position());
        JointVelocitySet.Assign(m_measured_js_pid.Velocity());

        // keep first two joint values as is
        mJointTrajectory.Goal.Ref(2, 0).Assign(m_setpoint_js_pid.Position().Ref(2, 0));
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
                Adapter.NeedEngage = false;
                Adapter.IsEngaged = true;
                mArmState.SetCurrentState("HOMED");
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

void mtsIntuitiveResearchKitPSM::EnterChangingCouplingTool(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    CouplingChange.Started = false;
    CouplingChange.CouplingForTool = true; // Load tool coupling
    if (Tool.NeedEngage) {
        CouplingChange.NextState = "ENGAGING_TOOL";
    } else {
        CouplingChange.NextState = "TOOL_ENGAGED";
    }
}

void mtsIntuitiveResearchKitPSM::EnterEngagingTool(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);

    // set PID limits for tool present
    UpdateConfigurationJointPID(true);

    // if for some reason we don't need to engage, basically, tool was
    // found before homing
    if (!Tool.NeedEngage) {
        mArmState.SetCurrentState("TOOL_ENGAGED");
        return;
    }

    // other case, initialize variables for tool engage
    EngagingStage = 1;
    LastEngagingStage = 4;
}

void mtsIntuitiveResearchKitPSM::RunEngagingTool(void)
{
    if (m_simulated) {
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
        SetPositionJointLocal(m_setpoint_js_pid.Position());
        // turn on PID
        PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
        PID.EnableTrackingError(true);

        // make sure we start from current state
        JointSet.Assign(m_setpoint_js_pid.Position());
        JointVelocitySet.Assign(m_measured_js_pid.Velocity());

        // check if the tool in outside the cannula
        if (m_measured_js_pid.Position().Element(2) >= mtsIntuitiveResearchKit::PSMOutsideCannula) {
            std::string message = this->GetName();
            message.append(": tool tip is outside the cannula, assuming it doesn't need to \"engage\".");
            message.append("  If the tool is not engaged properly, move the sterile adapter all the way up and re-insert the tool.");
            RobotInterface->SendStatus(message);
            mArmState.SetCurrentState("TOOL_ENGAGED");
        }

        // keep first three joint values as is
        mJointTrajectory.Goal.Ref(3, 0).Assign(m_setpoint_js_pid.Position().Ref(3, 0));
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
                Tool.NeedEngage = false;
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

void mtsIntuitiveResearchKitPSM::EnterToolEngaged(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);
    Tool.IsEngaged = true;
    // restore default PID tracking error
    PID.SetTrackingErrorTolerance(PID.DefaultTrackingErrorTolerance);
}

void mtsIntuitiveResearchKitPSM::TransitionToolEngaged(void)
{
    Tool.NeedEngage = false;
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("ENABLED");
    }
}

void mtsIntuitiveResearchKitPSM::EnterManual(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    PID.Enable(false);
}

void mtsIntuitiveResearchKitPSM::jaw_servo_jp(const prmPositionJointSet & jawPosition)
{
    // we need to need to at least ready to control in joint space
    if (!ArmIsReady("jaw_servo_jp", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (m_control_space) {
    case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
        if (! ((m_control_mode == mtsIntuitiveResearchKitArmTypes::POSITION_MODE)
               || (m_control_mode != mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE))) {
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
            // make sure all other joints have a reasonable cartesian
            // goal for all other joints
            CartesianSetParam.Goal().Assign(m_setpoint_cp.Position());
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
        if (m_control_mode != mtsIntuitiveResearchKitArmTypes::POSITION_MODE) {
            // we are initiating the control mode switch so we need to
            // set a reasonable JointSet
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
            // make sure all other joints have a reasonable goal
            JointSet.Assign(m_setpoint_js_pid.Position(), NumberOfJoints());
        }
        break;
    default:
        // we are initiating the control mode switch
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
        // make sure all other joints have a reasonable goal
        JointSet.Assign(m_setpoint_js_pid.Position(), NumberOfJoints());
    }

    // save goal
    JawGoal = jawPosition.Goal().at(0);
    m_new_pid_goal = true;
}

void mtsIntuitiveResearchKitPSM::jaw_move_jp(const prmPositionJointSet & jawPosition)
{
    // we need to need to at least ready to control in joint space
    if (!ArmIsReady("jaw_move_jp", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (m_control_space) {
    case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
        if (m_control_mode != mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
            // make sure all other joints have a reasonable goal
            mJointTrajectory.Goal.Assign(m_setpoint_js_pid.Position(), NumberOfJointsKinematics());
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
        if (m_control_mode != mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
            // make sure all other joints have a reasonable goal
            mJointTrajectory.Goal.Assign(m_setpoint_js_pid.Position(), NumberOfJointsKinematics());
        }
        break;
    default:
        // we are initiating the control mode switch
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
        // make sure all other joints have a reasonable goal
        mJointTrajectory.Goal.Assign(m_setpoint_js_pid.Position());
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
    if (m_operating_state.State() != prmOperatingState::ENABLED) {
        mtsIntuitiveResearchKitArm::SetPositionJointLocal(newPosition);
        return;
    }
    CMN_ASSERT(JointSetParam.Goal().size() == 7);
    JointSetParam.Goal().Zeros();
    ToJointsPID(newPosition, JointSetParam.Goal());
    JointSetParam.Goal().at(6) = JawGoal;
    JointSetParam.SetTimestamp(StateTable.GetTic());
    PID.servo_jp(JointSetParam);
}

void mtsIntuitiveResearchKitPSM::jaw_servo_jf(const prmForceTorqueJointSet & effort)
{
    if (!ArmIsReady("servo_jf", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (m_control_space) {
    case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
        if (m_control_mode != mtsIntuitiveResearchKitArmTypes::EFFORT_MODE) {
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);
            // make sure all other joints have a reasonable cartesian
            // goal
            mWrenchSet.Force().SetAll(0.0);
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
        if (m_control_mode != mtsIntuitiveResearchKitArmTypes::EFFORT_MODE) {
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
    if (mArmState.CurrentState() != "ENABLED") {
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
    PID.servo_jf(mTorqueSetParam);
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
    Adapter.IsEngaged = false;
    Adapter.IsPresent = present;
    if (present) {
        // we will need to engage this adapter
        Adapter.NeedEngage = true;
    } else {
        Adapter.NeedEngage = false;
        mArmState.SetCurrentState("HOMED");
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
    Tool.IsPresent = present;
    if (present) {
        // we will need to engage this tool
        Tool.NeedEngage = true;
        ToolEvents.ToolType(mtsIntuitiveResearchKitToolTypes::TypeToString(mToolType));
    } else {
        ToolEvents.ToolType(std::string());
    }
}

void mtsIntuitiveResearchKitPSM::SetToolType(const std::string & toolType)
{
    if (mToolTypeRequested || m_simulated) {
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
            RobotInterface->SendWarning(this->GetName() + ": tool type requested from user");
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

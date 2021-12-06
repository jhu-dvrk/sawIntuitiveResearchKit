/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2021 Johns Hopkins University (JHU), All Rights Reserved.

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
    mtsIntuitiveResearchKitArm(componentName, periodInSeconds),
    mToolList(*this)
{
    Init();
}

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsIntuitiveResearchKitArm(arg),
    mToolList(*this)
{
    Init();
}

void mtsIntuitiveResearchKitPSM::set_simulated(void)
{
    mtsIntuitiveResearchKitArm::set_simulated();
    // in simulation mode, we don't need clutch, adapter, tool IO nor Dallas
    RemoveInterfaceRequired("ManipClutch");
    RemoveInterfaceRequired("Adapter");
    RemoveInterfaceRequired("Tool");
    RemoveInterfaceRequired("Dallas");
}

void mtsIntuitiveResearchKitPSM::load_tool_list(const cmnPath & path,
                                                const std::string & indexFile)
{
    mToolList.Load(path, indexFile);
}

void mtsIntuitiveResearchKitPSM::tool_list_size(size_t & size) const
{
    size = mToolList.size();
}

void mtsIntuitiveResearchKitPSM::tool_name(const size_t & index, std::string & name) const
{
    name = mToolList.Name(index);
}

void mtsIntuitiveResearchKitPSM::tool_full_description(const size_t & index, std::string & description) const
{
    description = mToolList.FullDescription(index);
}

void mtsIntuitiveResearchKitPSM::PostConfigure(const Json::Value & jsonConfig,
                                               const cmnPath & configPath,
                                               const std::string & filename)
{
    // load default tool index
    load_tool_list(configPath);

    // extra tool definitions
    const auto jsonToolIndexFile = jsonConfig["custom-tool-index"];
    if (!jsonToolIndexFile.isNull()) {
        const auto toolIndexFile = jsonToolIndexFile.asString();
        auto fullname = configPath.Find(toolIndexFile);
        if (fullname == "") {
            CMN_LOG_CLASS_INIT_ERROR << "PostConfigure: " << this->GetName()
                                     << " using file \"" << filename << "\" can't find tool index file \""
                                     << toolIndexFile << "\" in path: "
                                     << configPath << std::endl;
            exit(EXIT_FAILURE);
        }
        load_tool_list(configPath, toolIndexFile);
    }

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
                const bool found = mToolList.Find(fixedTool, mToolIndex);
                if (!found) {
                    CMN_LOG_CLASS_INIT_ERROR << "PostConfigure: " << this->GetName()
                                             << ", \"" << fixedTool << "\" found in file \""
                                             << filename << "\" is not a supported type." << std::endl
                                             << "Supported tool types are:\n" << mToolList.PossibleNames("\n") << std::endl;
                    exit(EXIT_FAILURE);
                }
                // now look for the file to configure the tool
                mToolConfigured = ConfigureTool(mToolList.File(mToolIndex));
                if (!mToolConfigured) {
                    exit(EXIT_FAILURE);
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

bool mtsIntuitiveResearchKitPSM::ConfigureTool(const std::string & filename)
{
    std::string fullFilename;

    // try to locate the file based on tool type
    if (cmnPath::Exists(filename)) {
        fullFilename = filename;
    } else {
        // construct path using working directory and share/arm
        cmnPath path(cmnPath::GetWorkingDirectory());
        // find the file in tool
        path.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share/tool", cmnPath::TAIL);
        // find file if specified as share/<system>/...
        path.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share", cmnPath::TAIL);
        // finally, default installation directory
        path.Add(mtsIntuitiveResearchKit::DefaultInstallationDirectory + "/tool", cmnPath::TAIL);

        fullFilename = path.Find(filename);
        // still not found, try to add suffix to search again
        if (fullFilename == "") {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": failed to locate tool file for \""
                                     << filename << "\"" << std::endl;
            return false;
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
            return false;
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
            return false;
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
            return false;
        }

        // read 4x4 coupling for last 3 DOFs and jaws
        prmActuatorJointCoupling toolCoupling4;
        cmnDataJSON<prmActuatorJointCoupling>::DeSerializeText(toolCoupling4,
                                                               jsonCoupling);
        // build a coupling matrix for all 7 actuators/dofs
        CouplingChange.ToolCoupling
            .ActuatorToJointPosition().ForceAssign(vctDynamicMatrix<double>::Eye(number_of_joints()));
        // assign 4x4 matrix starting at position 3, 3
        CouplingChange.ToolCoupling
            .ActuatorToJointPosition().Ref(4, 4, 3, 3).Assign(toolCoupling4.ActuatorToJointPosition());

        // load jaw data, i.e. joint and torque limits
        const Json::Value jsonJaw = jsonConfig["jaw"];
        if (jsonJaw.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        }
        const Json::Value jsonJawQMin = jsonJaw["qmin"];
        if (jsonJawQMin.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::qmin\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        } else {
            CouplingChange.jaw_configuration_js.PositionMin().SetSize(1);
            CouplingChange.jaw_configuration_js.PositionMin().at(0) = jsonJawQMin.asDouble();
        }
        const Json::Value jsonJawQMax = jsonJaw["qmax"];
        if (jsonJawQMax.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::qmax\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
        } else {
            CouplingChange.jaw_configuration_js.PositionMax().SetSize(1);
            CouplingChange.jaw_configuration_js.PositionMax().at(0) = jsonJawQMax.asDouble();
        }
        const Json::Value jsonJawFTMax = jsonJaw["ftmax"];
        if (jsonJawFTMax.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": can find \"jaw::ftmax\" data in \"" << fullFilename << "\"" << std::endl;
            return false;
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
            return false;
        }
        // lower
        cmnDataJSON<vctDoubleVec>::DeSerializeText(CouplingChange.ToolEngageLowerPosition,
                                                   jsonEngagePosition["lower"]);
        if (CouplingChange.ToolEngageLowerPosition.size() != 4) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": \"tool-engage-position\" : \"lower\" must contain 4 elements in \""
                                     << fullFilename << "\"" << std::endl;
            return false;
        }
        // upper
        cmnDataJSON<vctDoubleVec>::DeSerializeText(CouplingChange.ToolEngageUpperPosition,
                                                   jsonEngagePosition["upper"]);
        if (CouplingChange.ToolEngageUpperPosition.size() != 4) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName()
                                     << ": \"tool-engage-position\" : \"upper\" must contain 4 elements in \""
                                     << fullFilename << "\"" << std::endl;
            return false;
        }

    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureTool " << this->GetName() << ": make sure the file \""
                                 << fullFilename << "\" is in JSON format" << std::endl;
        return false;
    }

    return true;
}

void mtsIntuitiveResearchKitPSM::UpdateStateJointKinematics(void)
{
    // if there is no tool, report joints as PID joints
    if (!IsCartesianReady()) {
        mtsIntuitiveResearchKitArm::UpdateStateJointKinematics();
        return;
    }

    const size_t nbPIDJoints = m_pid_measured_js.Name().size();
    const size_t jawIndex = nbPIDJoints - 1;

    if (m_jaw_measured_js.Name().size() == 0) {
        m_jaw_measured_js.Name().SetSize(1);
        m_jaw_measured_js.Name().at(0) = m_pid_measured_js.Name().at(jawIndex);
        m_jaw_measured_js.Position().SetSize(1);
        m_jaw_measured_js.Velocity().SetSize(1);
        m_jaw_measured_js.Effort().SetSize(1);

        m_jaw_setpoint_js.Name().SetSize(1);
        m_jaw_setpoint_js.Name().at(0) = m_pid_setpoint_js.Name().at(jawIndex);
        m_jaw_setpoint_js.Position().SetSize(1);
        m_jaw_setpoint_js.Velocity().SetSize(0);
        m_jaw_setpoint_js.Effort().SetSize(1);
    }

    m_jaw_measured_js.Position().at(0) = m_pid_measured_js.Position().at(jawIndex);
    m_jaw_measured_js.Velocity().at(0) = m_pid_measured_js.Velocity().at(jawIndex);
    m_jaw_measured_js.Effort().at(0)   = m_pid_measured_js.Effort().at(jawIndex);
    m_jaw_measured_js.Timestamp() = m_pid_measured_js.Timestamp();
    m_jaw_measured_js.Valid() = m_pid_measured_js.Valid();

    m_jaw_setpoint_js.Position().at(0) = m_pid_setpoint_js.Position().at(jawIndex);
    m_jaw_setpoint_js.Effort().at(0)   = m_pid_setpoint_js.Effort().at(jawIndex);
    m_jaw_setpoint_js.Timestamp() = m_pid_setpoint_js.Timestamp();
    m_jaw_setpoint_js.Valid() = m_pid_setpoint_js.Timestamp();

    if (!mSnakeLike) {

        // most tool, copy first n joints (6) from PID for kinematics
        // measured p/v/e
        m_kin_measured_js.Position().Assign(m_pid_measured_js.Position().Ref(number_of_joints_kinematics()));
        m_kin_measured_js.Velocity().Assign(m_pid_measured_js.Velocity().Ref(number_of_joints_kinematics()));
        m_kin_measured_js.Effort().Assign(m_pid_measured_js.Effort().Ref(number_of_joints_kinematics()));
        m_kin_measured_js.Timestamp() = m_pid_measured_js.Timestamp();
        m_kin_measured_js.Valid() = m_pid_measured_js.Valid();

        // setpoint p/e
        m_kin_setpoint_js.Position().Assign(m_pid_setpoint_js.Position().Ref(number_of_joints_kinematics()));
        m_kin_setpoint_js.Effort().Assign(m_pid_setpoint_js.Effort().Ref(number_of_joints_kinematics()));
        m_kin_setpoint_js.Timestamp() = m_pid_setpoint_js.Timestamp();
        m_kin_setpoint_js.Valid() = m_pid_setpoint_js.Valid();

    } else {

        // measured p/v/e
        m_kin_measured_js.Position().Assign(m_pid_measured_js.Position(), 4);
        m_kin_measured_js.Position().at(4) = m_kin_measured_js.Position().at(7) = m_pid_measured_js.Position().at(4) / 2.0;
        m_kin_measured_js.Position().at(5) = m_kin_measured_js.Position().at(6) = m_pid_measured_js.Position().at(5) / 2.0;

        m_kin_measured_js.Velocity().Assign(m_pid_measured_js.Velocity(), 4);
        m_kin_measured_js.Velocity().at(4) = m_kin_measured_js.Velocity().at(7) = m_pid_measured_js.Velocity().at(4) / 2.0;
        m_kin_measured_js.Velocity().at(5) = m_kin_measured_js.Velocity().at(6) = m_pid_measured_js.Velocity().at(5) / 2.0;

        m_kin_measured_js.Effort().Assign(m_pid_measured_js.Effort(), 4);
        m_kin_measured_js.Effort().at(4) = m_kin_measured_js.Effort().at(7) = m_pid_measured_js.Effort().at(4) / 2.0;
        m_kin_measured_js.Effort().at(5) = m_kin_measured_js.Effort().at(6) = m_pid_measured_js.Effort().at(5) / 2.0;
        m_kin_measured_js.Timestamp() = m_pid_measured_js.Timestamp();
        m_kin_measured_js.Valid() = m_pid_measured_js.Valid();

        // setpoint p/e
        m_kin_setpoint_js.Position().Assign(m_pid_setpoint_js.Position(), 4);
        m_kin_setpoint_js.Position().at(4) = m_kin_setpoint_js.Position().at(7) = m_pid_setpoint_js.Position().at(4) / 2.0;
        m_kin_setpoint_js.Position().at(5) = m_kin_setpoint_js.Position().at(6) = m_pid_setpoint_js.Position().at(5) / 2.0;

        m_kin_setpoint_js.Effort().Assign(m_pid_measured_js.Effort(), 4);
        m_kin_setpoint_js.Effort().at(4) = m_kin_setpoint_js.Effort().at(7) = m_pid_setpoint_js.Effort().at(4) / 2.0;
        m_kin_setpoint_js.Effort().at(5) = m_kin_setpoint_js.Effort().at(6) = m_pid_setpoint_js.Effort().at(5) / 2.0;
        m_kin_setpoint_js.Timestamp() = m_pid_setpoint_js.Timestamp();
        m_kin_setpoint_js.Valid() = m_pid_setpoint_js.Valid();
    }
}

void mtsIntuitiveResearchKitPSM::ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID)
{
    if (IsCartesianReady()) {
        // tool is present
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
    } else {
        // joint space, no tool yet so we can control all 7 actuators
        CMN_ASSERT(jointsKinematics.size() == 7);
        jointsPID.Assign(jointsKinematics);
    }
}

robManipulator::Errno mtsIntuitiveResearchKitPSM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal)
{
    // make sure we are away from RCM point, create a new goal on sphere around RCM point (i.e. origin)
    double distanceToRCM = cartesianGoal.Translation().Norm();
    double currentDepth = jointSet.at(2);

    // if too close to zero we're going to run into issue in any case
    if (distanceToRCM < 1.0 * cmn_mm) {
        m_arm_interface->SendWarning(GetName() + ": InverseKinematics, can't solve IK too close to RCM");
        return robManipulator::EFAILURE;
    }

    // IK
    robManipulator::Errno Err = Manipulator->InverseKinematics(jointSet, cartesianGoal);;

    // check equality constraint for snake like kinematic
    if (mSnakeLike) {
        // Check for equality Snake joints (4,7) and (5,6)
        if (fabs(jointSet.at(4) - jointSet.at(7)) > 0.00001 ||
            fabs(jointSet.at(5) - jointSet.at(6)) > 0.00001) {
            m_arm_interface->SendWarning(GetName() + ": InverseKinematics, equality constraint violated");
        }
    }

    // Find closest solution mod 2 Pi for roll along shaft
    if (Err == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = m_kin_measured_js.Position().at(3) - jointSet.at(3);
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet.at(3) = jointSet.at(3) + differenceInTurns * 2.0 * cmnPI;

        // project away from RCM if not safe, using axis at end of shaft
        vctFrm4x4 f4;
        if (Manipulator->links.size() >= 4) {
            f4 = Manipulator->ForwardKinematics(jointSet, 4);
        } else {
            f4 = Manipulator->ForwardKinematics(jointSet);
        }
        distanceToRCM = f4.Translation().Norm();

        // if not far enough, distance for axis 4 is fully determine by insertion joint so add to it
        if (distanceToRCM < mtsIntuitiveResearchKit::PSM::SafeDistanceFromRCM) {
            // two cases based in current depth, were we past min depth or not - to do this we need to compute the minimum depth using j2.
            const double minDepth = jointSet.at(2) + (mtsIntuitiveResearchKit::PSM::SafeDistanceFromRCM - distanceToRCM);
            // if we are already too close to RCM, simply prevent to get closer
            if (currentDepth <= minDepth) {
                jointSet.at(2) = std::max(currentDepth, jointSet.at(2));
            } else {
                // else, make sure we don't go deeper
                jointSet.at(2) = minDepth;
            }
        }
        return robManipulator::ESUCCESS;
    }

    return robManipulator::EFAILURE;
}

bool mtsIntuitiveResearchKitPSM::IsSafeForCartesianControl(void) const
{
    vctFrm4x4 f4;
    if (Manipulator->links.size() >= 4) {
        f4 = Manipulator->ForwardKinematics(m_kin_measured_js.Position(), 4);
    } else {
        f4 = Manipulator->ForwardKinematics(m_kin_measured_js.Position());
    }
    const double distanceToRCM = f4.Translation().Norm();
    return (distanceToRCM >=  (mtsIntuitiveResearchKit::PSM::SafeDistanceFromRCM
                               - mtsIntuitiveResearchKit::PSM::SafeDistanceFromRCMBuffer));
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
    m_trajectory_j.v_max.Ref(2, 0).SetAll(180.0 * cmnPI_180); // degrees per second
    m_trajectory_j.v_max.Element(2) = 0.2; // m per second
    m_trajectory_j.v_max.Ref(4, 3).SetAll(3.0 * 360.0 * cmnPI_180);
    m_trajectory_j.a_max.Ref(2, 0).SetAll(180.0 * cmnPI_180);
    m_trajectory_j.a_max.Element(2) = 0.2; // m per second
    m_trajectory_j.a_max.Ref(4, 3).SetAll(2.0 * 360.0 * cmnPI_180);
    m_trajectory_j.goal_tolerance.SetAll(3.0 * cmnPI_180); // hard coded to 3 degrees

    // default PID tracking errors
    PID.DefaultTrackingErrorTolerance.SetSize(number_of_joints());
    // first two rotations
    PID.DefaultTrackingErrorTolerance.Ref(2, 0).SetAll(20.0 * cmnPI_180); // 2 elements starting at 0 -> 0 1
    // translation
    PID.DefaultTrackingErrorTolerance.Element(2) = 20.0 * cmn_mm; // 20 mm -> 2
    // shaft rotation and tool orientation
    PID.DefaultTrackingErrorTolerance.Ref(3, 3).SetAll(35.0 * cmnPI_180); // 3 elements starting at 3 -> 3, 4, 5
    // jaws, allow more since we use PID large errors to apply large torques
    PID.DefaultTrackingErrorTolerance.Element(6) = 90.0 * cmnPI_180;

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
    CouplingChange.NoToolConfiguration.PositionMin().SetSize(number_of_joints());
    CouplingChange.NoToolConfiguration.PositionMax().SetSize(number_of_joints());
    CouplingChange.NoToolConfiguration.PositionMin().Assign(-91.0 * cmnPI_180,
                                                            -53.0 * cmnPI_180,
                                                            0.0 * cmn_mm,
                                                            -mtsIntuitiveResearchKit::PSM::AdapterEngageRange,
                                                            -mtsIntuitiveResearchKit::PSM::AdapterEngageRange,
                                                            -mtsIntuitiveResearchKit::PSM::AdapterEngageRange,
                                                            -mtsIntuitiveResearchKit::PSM::AdapterEngageRange);
    CouplingChange.NoToolConfiguration.PositionMax().Assign(91.0 * cmnPI_180,
                                                            53.0 * cmnPI_180,
                                                            240.0 * cmn_mm,
                                                            mtsIntuitiveResearchKit::PSM::AdapterEngageRange,
                                                            mtsIntuitiveResearchKit::PSM::AdapterEngageRange,
                                                            mtsIntuitiveResearchKit::PSM::AdapterEngageRange,
                                                            mtsIntuitiveResearchKit::PSM::AdapterEngageRange);
    // using values from sawControllersPID.xml: max_amp / nm_to_amp
    CouplingChange.NoToolConfiguration.EffortMin().SetSize(number_of_joints());
    CouplingChange.NoToolConfiguration.EffortMax().SetSize(number_of_joints());
    CouplingChange.NoToolConfiguration.EffortMax().Assign(3.316101,
                                                          3.316101,
                                                          2000.877926,
                                                          0.343642,
                                                          0.343642,
                                                          0.343642,
                                                          0.343642);
    CouplingChange.NoToolConfiguration.EffortMin().Assign(-CouplingChange.NoToolConfiguration.EffortMax());

    mtsInterfaceRequired * interfaceRequired;

    // Main interface should have been created by base class init
    CMN_ASSERT(m_arm_interface);
    m_jaw_measured_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_jaw_measured_js, "jaw/measured_js");

    m_jaw_setpoint_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_jaw_setpoint_js, "jaw/setpoint_js");

    // state table for configuration
    mStateTableConfiguration.AddData(CouplingChange.jaw_configuration_js, "jaw/configuration_js");

    // jaw interface
    m_arm_interface->AddCommandReadState(this->StateTable, m_jaw_measured_js, "jaw/measured_js");
    m_arm_interface->AddCommandReadState(this->StateTable, m_jaw_setpoint_js, "jaw/setpoint_js");
    m_arm_interface->AddCommandReadState(this->mStateTableConfiguration,
                                         CouplingChange.jaw_configuration_js, "jaw/configuration_js");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_servo_jp, this, "jaw/servo_jp");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_move_jp, this, "jaw/move_jp");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::jaw_servo_jf, this, "jaw/servo_jf");

    // tool specific interface
    m_arm_interface->AddCommandRead(&mtsIntuitiveResearchKitPSM::tool_list_size, this, "tool_list_size");
    m_arm_interface->AddCommandQualifiedRead(&mtsIntuitiveResearchKitPSM::tool_name, this, "tool_name");
    m_arm_interface->AddCommandQualifiedRead(&mtsIntuitiveResearchKitPSM::tool_full_description, this, "tool_full_description");

    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::set_adapter_present, this, "set_adapter_present");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::set_tool_present, this, "set_tool_present");
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitPSM::set_tool_type, this, "set_tool_type");
    m_arm_interface->AddEventWrite(ToolEvents.tool_type, "tool_type", std::string());
    m_arm_interface->AddEventVoid(ToolEvents.tool_type_request, "tool_type_request");

    m_arm_interface->AddEventWrite(ClutchEvents.ManipClutch, "ManipClutch", prmEventButton());

    CMN_ASSERT(PIDInterface);
    PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::CouplingEventHandler, this, "Coupling");
    PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EnableJointsEventHandler, this, "EnabledJoints");
    CouplingChange.LastEnabledJoints.SetSize(number_of_joints());
    CouplingChange.DesiredEnabledJoints.SetSize(number_of_joints());

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
        m_trajectory_j.goal.SetAll(0.0);
        m_trajectory_j.goal.at(2) = 12.0 * cmn_cm;
        return;
    }

    // check if tool is present and if user wants to go to zero position
    Tool.GetButton(Tool.IsPresent);
    if (m_homing_goes_to_zero
        && !Tool.IsPresent) {
        // move to zero position only there is no tool present
        m_trajectory_j.goal.SetAll(0.0);
    } else {
        // stay at current position by default
        m_trajectory_j.goal.Assign(m_pid_setpoint_js.Position());
    }
}

void mtsIntuitiveResearchKitPSM::TransitionHomed(void)
{
    if (!m_simulated) {
        Adapter.GetButton(Adapter.IsPresent);
        if (!Adapter.IsPresent) {
            Adapter.IsEngaged = false;
        }
    }
    if (!Adapter.IsEngaged &&
        (Adapter.IsPresent || m_simulated || Adapter.NeedEngage)) {
        mArmState.SetCurrentState("CHANGING_COUPLING_ADAPTER");
    }

    if (!m_simulated) {
        Tool.GetButton(Tool.IsPresent);
        if (!Tool.IsPresent) {
            Tool.IsEngaged = false;
        }
    }
    if (Adapter.IsEngaged && !Tool.IsEngaged) {
        if (!m_simulated) {
            if (Tool.IsPresent && mToolConfigured) {
                set_tool_present(true);
                mArmState.SetCurrentState("CHANGING_COUPLING_TOOL");
            }
        } else {
            // simulated case
            set_tool_present(true);
            // check if tool is configured, i.e. fixed or manual
            if (mToolConfigured) {
                mArmState.SetCurrentState("CHANGING_COUPLING_TOOL");
            } else {
                // request tool type if needed
                if (!mToolTypeRequested) {
                    mToolTypeRequested = true;
                    ToolEvents.tool_type_request();
                    m_arm_interface->SendWarning(this->GetName() + ": tool type requested from user");
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

    const double currentTime = this->StateTable.GetTic();

    // first phase, disable last 4 joints and wait
    if (!CouplingChange.Started) {
        // keep first 3 on
        CouplingChange.DesiredEnabledJoints.Ref(3, 0).SetAll(true);
        // turn off last 4
        CouplingChange.DesiredEnabledJoints.Ref(4, 3).SetAll(false);
        PID.EnableJoints(CouplingChange.DesiredEnabledJoints);
        CouplingChange.WaitingForEnabledJoints = true;
        m_homing_timer = currentTime;
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
                    const vctDoubleMat identity(vctDoubleMat::Eye(number_of_joints()));
                    // set desired
                    CouplingChange.DesiredCoupling.Assign(identity);
                }
                PID.SetCoupling(CouplingChange.DesiredCoupling);
                CouplingChange.WaitingForEnabledJoints = false;
                CouplingChange.WaitingForCoupling = true;
                CouplingChange.ReceivedCoupling = false;
                return;
            } else {
                // check time
                const double timeToEnableJoints = 30.0 * cmn_s; // large timeout
                if ((currentTime - m_homing_timer) > timeToEnableJoints) {
                    m_arm_interface->SendError(this->GetName() + ": can't disable last four axes to change coupling.");
                    CMN_LOG_CLASS_RUN_ERROR << "RunChangingCoupling: " << this->GetName()
                                            << "\nexpecting enabled joints:" << CouplingChange.DesiredEnabledJoints
                                            << "\nbut received: " << CouplingChange.LastEnabledJoints
                                            << std::endl;
                    SetDesiredState("FAULT");
                } else {
                    m_arm_interface->SendStatus(this->GetName() + ": waiting to disable last four axes to change coupling.");
                }
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
                m_arm_interface->SendError(this->GetName() + ": can't set coupling.");
                SetDesiredState("FAULT");
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

        // just to be absolutely totally sure
        CMN_ASSERT(number_of_joints() == 7);

        // at most we use 6 joints from kinematics, 7th is always the
        // jaws.  PID is always 6+1, kinematics is usually 6 (most ISI
        // tools) except 8 for 5mm tools (snake like).  User defined
        // kinematics might be less than 6.
        const size_t jawIndex = 6;
        const size_t nbJointsFromKinematics = std::min(number_of_joints_kinematics(),
                                                       static_cast<size_t>(6));

        // get names, types and joint limits for kinematics config from the manipulator
        // name and types need conversion
        mStateTableConfiguration.Start();
        m_pid_configuration_js.Name().SetSize(number_of_joints());
        m_pid_configuration_js.Type().SetSize(number_of_joints());
        std::vector<std::string> names(number_of_joints_kinematics());
        std::vector<robJoint::Type> types(number_of_joints_kinematics());
        this->Manipulator->GetJointNames(names);
        this->Manipulator->GetJointTypes(types);
        for (size_t index = 0; index < nbJointsFromKinematics; ++index) {
            m_pid_configuration_js.Name().at(index) = names.at(index);
            switch (types.at(index)) {
            case robJoint::HINGE:
                m_pid_configuration_js.Type().at(index) = PRM_JOINT_REVOLUTE;
                break;
            case robJoint::SLIDER:
                m_pid_configuration_js.Type().at(index) = PRM_JOINT_PRISMATIC;
                break;
            default:
                m_pid_configuration_js.Type().at(index) = PRM_JOINT_UNDEFINED;
                break;
            }
        }

        // limits need to take into account snake case
        vctDoubleVec lowerFromKinematics(number_of_joints_kinematics());
        vctDoubleVec upperFromKinematics(number_of_joints_kinematics());

        m_pid_configuration_js.PositionMin().SetSize(number_of_joints());
        m_pid_configuration_js.PositionMax().SetSize(number_of_joints());
        m_pid_configuration_js.EffortMin().SetSize(number_of_joints());
        m_pid_configuration_js.EffortMax().SetSize(number_of_joints());

        // position limits
        Manipulator->GetJointLimits(lowerFromKinematics,
                                    upperFromKinematics);
        // use kinematic joints... all but last
        m_pid_configuration_js.PositionMin().Ref(nbJointsFromKinematics).Assign(lowerFromKinematics.Ref(nbJointsFromKinematics));
        m_pid_configuration_js.PositionMax().Ref(nbJointsFromKinematics).Assign(upperFromKinematics.Ref(nbJointsFromKinematics));
        if (mSnakeLike) {
            // add kinematic joint limits
            m_pid_configuration_js.PositionMin().at(4) += lowerFromKinematics.at(7);
            m_pid_configuration_js.PositionMin().at(5) += lowerFromKinematics.at(6);
            m_pid_configuration_js.PositionMax().at(4) += upperFromKinematics.at(7);
            m_pid_configuration_js.PositionMax().at(5) += upperFromKinematics.at(6);
        }
        // ...and jaw
        m_pid_configuration_js.PositionMin().at(jawIndex) = CouplingChange.jaw_configuration_js.PositionMin().at(0);
        m_pid_configuration_js.PositionMax().at(jawIndex) = CouplingChange.jaw_configuration_js.PositionMax().at(0);

        // force torque
        Manipulator->GetFTMaximums(upperFromKinematics);
        // use kinematic joints... all but last
        m_pid_configuration_js.EffortMax().Ref(nbJointsFromKinematics).Assign(upperFromKinematics.Ref(nbJointsFromKinematics));
        if (mSnakeLike) {
            // add kinematic joint limits
            m_pid_configuration_js.EffortMax().at(4) += upperFromKinematics.at(7);
            m_pid_configuration_js.EffortMax().at(5) += upperFromKinematics.at(6);
        }
        // ...and jaw
        m_pid_configuration_js.EffortMax().at(jawIndex) = CouplingChange.jaw_configuration_js.EffortMax().at(0);
        m_pid_configuration_js.EffortMin().ProductOf(-1.0, m_pid_configuration_js.EffortMax()); // manipulator assumes symmetry

        mStateTableConfiguration.Advance();

        // set
        PID.configure_js(m_pid_configuration_js);
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
        vctDoubleVec tolerances(number_of_joints());
        // first two rotations and translation, in case someone is pushing/holding arm
        tolerances.Ref(2, 0).SetAll(10.0 * cmnPI_180); // 10 degrees
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        // tool/adapter gears should have little resistance?
        tolerances.Ref(4, 3).SetAll(45.0 * cmnPI_180);
        PID.SetTrackingErrorTolerance(tolerances);
        servo_jp_internal(m_pid_setpoint_js.Position());
        // turn on PID
        PID.EnableJoints(vctBoolVec(number_of_joints(), true));
        PID.EnableTrackingError(true);

        // make sure we start from current state
        m_servo_jp.Assign(m_pid_setpoint_js.Position());
        m_servo_jv.Assign(m_pid_measured_js.Velocity());

        // keep first two joint values as is
        m_trajectory_j.goal.Ref(2, 0).Assign(m_pid_setpoint_js.Position().Ref(2, 0));
        // sterile adapter should be raised up
        m_trajectory_j.goal[2] = 0.0;
        // set last 4 to -170.0
        m_trajectory_j.goal.Ref(4, 3).SetAll(-mtsIntuitiveResearchKit::PSM::AdapterEngageRange);
        m_trajectory_j.goal_v.SetAll(0.0);
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
        control_move_jp_on_start();
        EngagingStage = 2;
        return;
    }

    m_trajectory_j.Reflexxes.Evaluate(m_servo_jp,
                                      m_servo_jv,
                                      m_trajectory_j.goal,
                                      m_trajectory_j.goal_v);
    servo_jp_internal(m_servo_jp);

    const robReflexxes::ResultType trajectoryResult = m_trajectory_j.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (m_trajectory_j.end_time == 0.0) {
            m_trajectory_j.end_time = currentTime + m_trajectory_j.Reflexxes.Duration();
            m_homing_timer = m_trajectory_j.end_time;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        {
            // check if we were in last phase
            if (EngagingStage > LastEngagingStage) {
                Adapter.NeedEngage = false;
                Adapter.IsEngaged = true;
                control_move_jp_on_stop(true); // goal reached
                mArmState.SetCurrentState("HOMED");
            } else {
                if (EngagingStage != LastEngagingStage) {
                    m_trajectory_j.goal.Ref(4, 3) *= -1.0; // toggle back and forth
                } else {
                    m_trajectory_j.goal.Ref(4, 3).SetAll(0.0); // back to zero position
                }
                m_trajectory_j.end_time = 0.0;
                std::stringstream message;
                message << this->GetName() << ": engaging adapter " << EngagingStage - 1 << " of " << LastEngagingStage - 1;
                m_arm_interface->SendStatus(message.str());
                EngagingStage++;
            }
        }
        break;

    default:
        m_arm_interface->SendError(this->GetName() + ": error while evaluating trajectory");
        SetDesiredState("FAULT");
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
        vctDoubleVec tolerances(number_of_joints());
        // first two rotations and translation, in case someone is pushing/holding arm
        tolerances.Ref(2, 0).SetAll(10.0 * cmnPI_180); // 10 degrees
        tolerances.Element(2) = 10.0 * cmn_mm; // 10 mm
        // tool/adapter gears should have little resistance?
        tolerances.Ref(4, 3).SetAll(45.0 * cmnPI_180);
        PID.SetTrackingErrorTolerance(tolerances);
        servo_jp_internal(m_pid_setpoint_js.Position());
        // turn on PID
        PID.EnableJoints(vctBoolVec(number_of_joints(), true));
        PID.EnableTrackingError(true);

        // make sure we start from current state
        m_servo_jp.Assign(m_pid_setpoint_js.Position());
        m_servo_jv.Assign(m_pid_measured_js.Velocity());

        // check if the tool in outside the cannula
        if (m_pid_measured_js.Position().Element(2) >= mEngageDepth) {
            std::string message = this->GetName();
            message.append(": tool tip is outside the cannula, assuming it doesn't need to \"engage\".");
            message.append("  If the tool is not engaged properly, move the sterile adapter all the way up and re-insert the tool.");
            m_arm_interface->SendStatus(message);
            mArmState.SetCurrentState("TOOL_ENGAGED");
        }

        // keep first three joint values as is
        m_trajectory_j.goal.Ref(3, 0).Assign(m_pid_setpoint_js.Position().Ref(3, 0));
        // set last 4 to user preferences
        m_trajectory_j.goal.Ref(4, 3).Assign(CouplingChange.ToolEngageLowerPosition);
        m_trajectory_j.goal_v.SetAll(0.0);
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                               mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
        control_move_jp_on_start();
        EngagingStage = 2;
        return;
    }

    m_trajectory_j.Reflexxes.Evaluate(m_servo_jp,
                                      m_servo_jv,
                                      m_trajectory_j.goal,
                                      m_trajectory_j.goal_v);
    servo_jp_internal(m_servo_jp);


    const robReflexxes::ResultType trajectoryResult = m_trajectory_j.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (m_trajectory_j.end_time == 0.0) {
            m_trajectory_j.end_time = currentTime + m_trajectory_j.Reflexxes.Duration();
            m_homing_timer = m_trajectory_j.end_time;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        {
            // check if we were in last phase
            if (EngagingStage > LastEngagingStage) {
                Tool.NeedEngage = false;
                control_move_jp_on_stop(true); // goal reached
                mArmState.SetCurrentState("TOOL_ENGAGED");
            } else {
                if (EngagingStage != LastEngagingStage) {
                    // toggle between lower and upper
                    if (EngagingStage % 2 == 0) {
                        m_trajectory_j.goal.Ref(4, 3).Assign(CouplingChange.ToolEngageUpperPosition);
                    } else {
                        m_trajectory_j.goal.Ref(4, 3).Assign(CouplingChange.ToolEngageLowerPosition);
                    }
                } else {
                    m_trajectory_j.goal.Ref(4, 3).SetAll(0.0); // back to zero position
                }
                m_trajectory_j.end_time = 0.0;
                std::stringstream message;
                message << this->GetName() << ": engaging tool " << EngagingStage - 1 << " of " << LastEngagingStage - 1;
                m_arm_interface->SendStatus(message.str());
                EngagingStage++;
            }
        }
        break;

    default:
        m_arm_interface->SendError(this->GetName() + " error while evaluating trajectory.");
        SetDesiredState("FAULT");
        break;
    }
}

void mtsIntuitiveResearchKitPSM::EnterToolEngaged(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);
    Tool.IsEngaged = true;
    // restore default PID tracking error
    PID.SetTrackingErrorTolerance(PID.DefaultTrackingErrorTolerance);
    // resize kinematics vectors
    const size_t numberOfKinematicsJoints = this->Manipulator->links.size();
    m_kin_measured_js.Name().SetSize(numberOfKinematicsJoints);
    m_kin_measured_js.Name().Assign(m_kin_configuration_js.Name());
    m_kin_measured_js.Position().SetSize(numberOfKinematicsJoints);
    m_kin_measured_js.Velocity().SetSize(numberOfKinematicsJoints);
    m_kin_measured_js.Effort().SetSize(numberOfKinematicsJoints);
    m_kin_setpoint_js.Name().SetSize(numberOfKinematicsJoints);
    m_kin_setpoint_js.Name().Assign(m_kin_configuration_js.Name());
    m_kin_setpoint_js.Position().SetSize(numberOfKinematicsJoints);
    m_kin_setpoint_js.Effort().SetSize(numberOfKinematicsJoints);
}

void mtsIntuitiveResearchKitPSM::TransitionToolEngaged(void)
{
    Tool.NeedEngage = false;
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("HOMED");
    }
}

void mtsIntuitiveResearchKitPSM::EnterManual(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    PID.Enable(false);
}

void mtsIntuitiveResearchKitPSM::jaw_servo_jp(const prmPositionJointSet & jawPosition)
{
    // we need to need to at least ready to control in cartesian space
    if (!ArmIsReady("jaw_servo_jp", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // keep cartesian space is already there, otherwise use joint_space
    switch (m_control_space) {
    case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
        if (! (m_control_mode == mtsIntuitiveResearchKitArmTypes::POSITION_MODE)) {
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
            // make sure all other joints have a reasonable cartesian
            // goal for all other joints
            m_servo_cp.Goal().Assign(m_setpoint_cp.Position());
        }
        break;
    default:
        if (! (m_control_mode == mtsIntuitiveResearchKitArmTypes::POSITION_MODE)) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
            // make sure all other joints have a reasonable goal
            m_servo_jp.Assign(m_pid_setpoint_js.Position(), number_of_joints());
        }
    }

    // save goal
    m_jaw_servo_jp = jawPosition.Goal().at(0);
    m_new_pid_goal = true;
}

void mtsIntuitiveResearchKitPSM::jaw_move_jp(const prmPositionJointSet & jawPosition)
{
    // we need to need to at least ready to control in cartesian space
    if (!ArmIsReady("jaw_move_jp", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
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
            m_trajectory_j.goal.Assign(m_pid_setpoint_js.Position(), number_of_joints_kinematics());
        }
        break;
    default:
        if (m_control_mode != mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE) {
            // we are initiating the control mode switch
            SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                                   mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
            // make sure all other joints have a reasonable goal
            m_trajectory_j.goal.Assign(m_pid_setpoint_js.Position());
        }
    }

    // force trajectory re-evaluation with new goal for last joint
    control_move_jp_on_start();
    m_trajectory_j.goal[6] = jawPosition.Goal().at(0);

    // save position jaw goal, this might lead to jump if the user
    // interupts the jaw trajectory
    m_jaw_servo_jp = jawPosition.Goal().at(0);
}

void mtsIntuitiveResearchKitPSM::servo_jp_internal(const vctDoubleVec & newPosition)
{
    if (!IsCartesianReady()) {
        mtsIntuitiveResearchKitArm::servo_jp_internal(newPosition);
        return;
    }
    CMN_ASSERT(m_servo_jp_param.Goal().size() == 7);
    m_servo_jp_param.Goal().Zeros();
    ToJointsPID(newPosition, m_servo_jp_param.Goal());
    m_servo_jp_param.Goal().at(6) = m_jaw_servo_jp;
    m_servo_jp_param.SetTimestamp(StateTable.GetTic());
    PID.servo_jp(m_servo_jp_param);
}

void mtsIntuitiveResearchKitPSM::jaw_servo_jf(const prmForceTorqueJointSet & effort)
{
    if (!ArmIsReady("servo_jf", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
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
            m_cf_set.Force().SetAll(0.0);
        }
        break;
    default:
        // we are initiating the control mode switch
        SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                               mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);
        // make sure all other joints have a reasonable goal
        mEffortJointSet.ForceTorque().SetAll(0.0);
    }

    // save the desired effort
    m_jaw_servo_jf = effort.ForceTorque().at(0);
}

void mtsIntuitiveResearchKitPSM::servo_jf_internal(const vctDoubleVec & newEffort)
{
    if (!IsCartesianReady()) {
        mtsIntuitiveResearchKitArm::servo_jf_internal(newEffort);
        return;
    }

    // pad array for PID
    vctDoubleVec torqueDesired(number_of_joints(), 0.0); // for PID
    if (mSnakeLike) {
        std::cerr << CMN_LOG_DETAILS << " need to convert 8 joints from snake to 6 for force control" << std::endl;
    } else {
        torqueDesired.Assign(newEffort, number_of_joints_kinematics());
    }
    // add torque for jaws
    torqueDesired.at(6) = m_jaw_servo_jf;

    // convert to cisstParameterTypes
    mTorqueSetParam.SetForceTorque(torqueDesired);
    mTorqueSetParam.SetTimestamp(StateTable.GetTic());
    PID.servo_jf(mTorqueSetParam);
}

void mtsIntuitiveResearchKitPSM::control_move_jp_on_stop(const bool goal_reached)
{
    if (IsCartesianReady()) {
        // save end position as starting servo for jaws
        m_jaw_servo_jp = m_servo_jp_param.Goal().at(6);
    }
    mtsIntuitiveResearchKitArm::control_move_jp_on_stop(goal_reached);
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
    if (CouplingChange.WaitingForEnabledJoints) {
        CouplingChange.ReceivedEnabledJoints = true;
        CouplingChange.LastEnabledJoints.Assign(enable);
    }
}

void mtsIntuitiveResearchKitPSM::set_adapter_present(const bool & present)
{
    Adapter.IsEngaged = false;
    Adapter.IsPresent = present;
    if (present) {
        // we will need to engage this adapter
        Adapter.NeedEngage = true;
    } else {
        Adapter.NeedEngage = false;
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerAdapter(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        set_adapter_present(true);
        break;
    case prmEventButton::RELEASED:
        set_adapter_present(false);
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitPSM::set_tool_present(const bool & present)
{
    Tool.IsPresent = present;
    if (present) {
        // we will need to engage this tool
        Tool.NeedEngage = true;
        ToolEvents.tool_type(mToolList.Name(mToolIndex));
    } else {
        ToolEvents.tool_type(std::string());
    }
}

void mtsIntuitiveResearchKitPSM::set_tool_type(const std::string & toolType)
{
    if (mToolTypeRequested || m_simulated) {
        EventHandlerToolType(toolType);
        if (mToolConfigured) {
            mToolTypeRequested = false;
        }
    } else {
        m_arm_interface->SendWarning(this->GetName() + ": received request to set tool type but not expecting it now.  Request ignored.");
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
            ToolEvents.tool_type_request();
            m_arm_interface->SendWarning(this->GetName() + ": tool type requested from user");
            break;
        case mtsIntuitiveResearchKitToolTypes::FIXED:
            set_tool_present(true);
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
            set_tool_present(false);
            break;
        case mtsIntuitiveResearchKitToolTypes::FIXED:
            set_tool_present(false);
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
    m_arm_interface->SendStatus(this->GetName() + ": setting up for tool type \"" + toolType + "\"");
    // check if the tool is in the supported list
    if (!mToolList.Find(toolType, mToolIndex)) {
        CMN_LOG_CLASS_RUN_ERROR << "Supported tool types are:\n" << mToolList.PossibleNames("\n") << std::endl;
        m_arm_interface->SendError(this->GetName() + ": tool type \""
                                   + toolType + "\" is not supported, see cisstLog for details");
        ToolEvents.tool_type(std::string("ERROR"));
        return;
    }
    // supported tools
    const std::string toolFile = mToolList.File(mToolIndex);
    m_arm_interface->SendStatus(this->GetName() + ": using tool file \"" + toolFile
                                + "\" for: " + mToolList.FullDescription(mToolIndex));
    mToolConfigured = ConfigureTool(toolFile);
    if (mToolConfigured) {
        set_tool_present(true);
        if (mToolList.Generation(mToolIndex) == "S") {
            mEngageDepth = mtsIntuitiveResearchKit::PSM::EngageDepthS;
        } else {
            mEngageDepth = mtsIntuitiveResearchKit::PSM::EngageDepthClassic;
        }
    } else {
        m_arm_interface->SendError(this->GetName() + ": failed to configure tool \"" + toolType + "\", check terminal output and cisstLog file");
        ToolEvents.tool_type(std::string("ERROR"));
    }
}

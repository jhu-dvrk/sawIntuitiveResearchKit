/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Brendan Burkhart
  Created on: 2025-01-23

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

// header
#include "mtsBilateralTeleOperationPSM.h"

// cisst includes
#include <cisstMultiTask/mtsManagerLocal.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsBilateralTeleOperationPSM,
                                      mtsTeleOperationPSM,
                                      mtsTaskPeriodicConstructorArg);

void mtsBilateralTeleOperationPSM::ForceSource::Configure(mtsBilateralTeleOperationPSM* teleop, const Json::Value & jsonConfig) {
    std::string component_name;
    std::string provided_interface_name;
    std::string function_name;

    Json::Value value;

    value = jsonConfig["component"];
    if (!value.empty()) {
        component_name = value.asString();
    }

    value = jsonConfig["interface"];
    if (!value.empty()) {
        provided_interface_name = value.asString();
    }

    value = jsonConfig["function"];
    if (!value.empty()) {
        function_name = value.asString();
    }

    std::string required_interface_name = provided_interface_name + "-force-source";
    mtsInterfaceRequired* interface = teleop->AddInterfaceRequired(required_interface_name);
    if (interface) {
        interface->AddFunction(function_name, measured_cf);
    }

    mtsManagerLocal* manager = mtsComponentManager::GetInstance();
    manager->Connect(component_name, provided_interface_name,
                     teleop->GetName(), required_interface_name);
}

void mtsBilateralTeleOperationPSM::Arm::populateInterface(mtsInterfaceRequired* interface)
{
    interface->AddFunction("servo_cpvf", servo_cpvf, MTS_OPTIONAL);
    interface->AddFunction("measured_cs", measured_cs, MTS_OPTIONAL);
}

prmStateCartesian mtsBilateralTeleOperationPSM::Arm::computeGoal(Arm* target, double scale)
{
    prmStateCartesian goal;
    prmStateCartesian target_state = target->state();

    vct3 target_translation = target_state.Position().Translation() - target->ClutchOrigin().Translation();
    vct3 goal_translation = scale * target_translation + ClutchOrigin().Translation();

    auto align = vctMatRot3(target->ClutchOrigin().Rotation().TransposeRef() * ClutchOrigin().Rotation());

    if (target_state.PositionIsValid()) {
        goal.Position().Translation() = goal_translation;
        goal.Position().Rotation() = target_state.Position().Rotation() * align;
    }
    goal.PositionIsValid() = target_state.PositionIsValid();

    if (target_state.VelocityIsValid()) {
        goal.Velocity().Ref<3>(0) = scale * target_state.Velocity().Ref<3>(0);
        goal.Velocity().Ref<3>(3) = target_state.Velocity().Ref<3>(3);
    }
    goal.VelocityIsValid() = target_state.VelocityIsValid();

    prmStateCartesian current_state = state();
    if (target_state.ForceIsValid() && current_state.ForceIsValid()) {
        goal.Force() = -target_state.Force() - current_state.Force();
    }
    goal.ForceIsValid() = target_state.ForceIsValid() && current_state.ForceIsValid();

    return goal;
}

prmStateCartesian mtsBilateralTeleOperationPSM::Arm::state()
{
    prmStateCartesian measured_state;
    measured_cs(measured_state);

    if (force_source) {
        force_source->measured_cf(force_source->m_measured_cf);
        measured_state.Force() = force_source->m_measured_cf.Force();
        measured_state.ForceIsValid() = force_source->m_measured_cf.Valid();
    }

    return measured_state;
}

void mtsBilateralTeleOperationPSM::Arm::servo(prmStateCartesian goal)
{
    servo_cpvf(goal);
}

vctFrm4x4& mtsBilateralTeleOperationPSM::ArmMTM::ClutchOrigin() { return teleop->mMTM.CartesianInitial; }

prmStateCartesian mtsBilateralTeleOperationPSM::ArmMTM::state()
{
    if (measured_cs.IsValid()) {
        return Arm::state();
    }

    // measured_cs not available, fall back to measured_cp/measured_cv
    prmStateCartesian state;
    state.Position() = teleop->mMTM.m_measured_cp.Position();
    state.PositionIsValid() = teleop->mMTM.m_measured_cp.Valid();

    if (teleop->mMTM.use_measured_cv) {
        auto mtm_velocity = teleop->mMTM.m_measured_cv;
        state.Velocity().Ref<3>(0) = mtm_velocity.VelocityLinear();
        state.Velocity().Ref<3>(3) = mtm_velocity.VelocityAngular();
        state.VelocityIsValid() = mtm_velocity.Valid();
    } else {
        state.VelocityIsValid() = false;
    }

    state.ForceIsValid() = false;

    return state;
}

void mtsBilateralTeleOperationPSM::ArmMTM::servo(prmStateCartesian goal)
{
    // Use servo_cpvf if available, otherwise fall back to servo_cp
    if (servo_cpvf.IsValid()) {
        Arm::servo(goal);
    } else {
        prmPositionCartesianSet& servo = teleop->mArmMTM.m_servo_cp;
        servo.Goal() = goal.Position();

        if (goal.VelocityIsValid()) {
            servo.Velocity() = goal.Velocity().Ref<3>(0);
            servo.VelocityAngular() = goal.Velocity().Ref<3>(3);
        } else {
            servo.Velocity().Assign(vct3(0));
            servo.VelocityAngular().Assign(vct3(0));
        }

        teleop->mArmMTM.servo_cp(servo);
    }
}

vctFrm4x4& mtsBilateralTeleOperationPSM::ArmPSM::ClutchOrigin() { return teleop->mPSM.CartesianInitial; };

prmStateCartesian mtsBilateralTeleOperationPSM::ArmPSM::state()
{
    if (measured_cs.IsValid()) {
        return Arm::state();
    }

    // measured_cs not available, fall back to measured_cp/measured_cv
    
    prmStateCartesian state;
    teleop->mArmPSM.measured_cp(teleop->mArmPSM.m_measured_cp);
    state.Position() = teleop->mArmPSM.m_measured_cp.Position();
    state.PositionIsValid() = teleop->mArmPSM.m_measured_cp.Valid();

    teleop->mArmPSM.measured_cv(teleop->mArmPSM.m_measured_cv);
    auto psm_velocity = teleop->mArmPSM.m_measured_cv;
    state.Velocity().Ref<3>(0) = psm_velocity.VelocityLinear();
    state.Velocity().Ref<3>(3) = psm_velocity.VelocityAngular();
    state.VelocityIsValid() = psm_velocity.Valid();

    state.ForceIsValid() = false;

    return state;
}

void mtsBilateralTeleOperationPSM::ArmPSM::servo(prmStateCartesian goal)
{
    // Use servo_cpvf if available, otherwise fall back to servo_cp
    if (servo_cpvf.IsValid()) {
        Arm::servo(goal);
    } else {
        prmPositionCartesianSet& servo = teleop->mPSM.m_servo_cp;
        servo.Goal() = goal.Position();

        if (goal.VelocityIsValid()) {
            servo.Velocity() = goal.Velocity().Ref<3>(0);
            servo.VelocityAngular() = goal.Velocity().Ref<3>(3);
        } else {
            servo.Velocity().Assign(vct3(0));
            servo.VelocityAngular().Assign(vct3(0));
        }

        teleop->mPSM.servo_cp(servo);
    }
}

mtsBilateralTeleOperationPSM::mtsBilateralTeleOperationPSM(const std::string & componentName,
                                                       const double periodInSeconds) :
    mtsTeleOperationPSM(componentName, periodInSeconds), mArmMTM(this), mArmPSM(this) { Init(); }

mtsBilateralTeleOperationPSM::mtsBilateralTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg) :
    mtsTeleOperationPSM(arg), mArmMTM(this), mArmPSM(this) { Init(); }

void mtsBilateralTeleOperationPSM::Init() {
    m_bilateral_enabled = true;

    mtsInterfaceRequired* interface;

    interface = GetInterfaceRequired("MTM");
    if (interface) {
        interface->AddFunction("servo_cp", mArmMTM.servo_cp);
        mArmMTM.populateInterface(interface);
    }

    interface = GetInterfaceRequired("PSM");
    if (interface) {
        interface->AddFunction("measured_cp", mArmPSM.measured_cp);
        mArmPSM.populateInterface(interface);
    }

    mConfigurationStateTable->AddData(m_bilateral_enabled, "bilateral_enabled");

    mtsInterfaceProvided* setting_interface = GetInterfaceProvided("Setting");
    if (setting_interface) {
        setting_interface->AddCommandWrite(&mtsBilateralTeleOperationPSM::set_bilateral_enabled, this,
                                        "set_bilateral_enabled", m_bilateral_enabled);
        setting_interface->AddCommandReadState(*(mConfigurationStateTable),
                                        m_bilateral_enabled, "bilateral_enabled");
        setting_interface->AddEventWrite(bilateral_enabled_event,
                                    "bilateral_enabled", m_bilateral_enabled);
    }
}

void mtsBilateralTeleOperationPSM::Configure(const Json::Value & jsonConfig)
{
    mtsTeleOperationPSM::Configure(jsonConfig);
    Json::Value jsonValue;

    jsonValue = jsonConfig["psm_force_source"];
    if (!jsonValue.empty()) {
        auto source = std::make_unique<ForceSource>();
        source->Configure(this, jsonValue);
        mArmPSM.add_force_source(std::move(source));
    }

    jsonValue = jsonConfig["mtm_force_source"];
    if (!jsonValue.empty()) {
        auto source = std::make_unique<ForceSource>();
        source->Configure(this, jsonValue);
        mArmMTM.add_force_source(std::move(source));
    }

    jsonValue = jsonConfig["mtm_torque_gain"];
    if (!jsonValue.empty()) {
        m_mtm_torque_gain = jsonValue.asDouble();
    }
}

void mtsBilateralTeleOperationPSM::set_bilateral_enabled(const bool & enabled)
{
    mConfigurationStateTable->Start();
    m_bilateral_enabled = enabled;
    mConfigurationStateTable->Advance();
    bilateral_enabled_event(m_bilateral_enabled);
}

void mtsBilateralTeleOperationPSM::RunCartesianTeleop()
{
    // fall back to default behavior when in unilateral mode
    if (!m_bilateral_enabled) {
        mtsTeleOperationPSM::RunCartesianTeleop();
        return;
    }

    if (m_clutched) {
        return;
    }

    auto psm_goal = mArmPSM.computeGoal(&mArmMTM, m_scale);
    mArmPSM.servo(psm_goal);

    auto mtm_goal = mArmMTM.computeGoal(&mArmPSM, 1.0 / m_scale);
    // scale MTM torque goal to reduce oscillations
    mtm_goal.Force().Ref<3>(3) = m_mtm_torque_gain * mtm_goal.Force().Ref<3>(3);
    mArmMTM.servo(mtm_goal);
}

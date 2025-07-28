/*
  Author(s):  Brendan Burkhart
  Created on: 2025-07-06

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "console_inputs_editor.hpp"

#include "models/config_model.hpp"
#include "models/list_model.hpp"

namespace system_wizard {

ConsoleInputsEditor::ConsoleInputsEditor(ConsoleInputConfig& model, ListModelT<ArmConfig>& arms, QWidget* parent)
    : QWidget(parent), model(&model), arms(&arms)
{
    QObject::connect(this->arms, &ListModel::updated, this, &ConsoleInputsEditor::updateAvailableArms);

    QVBoxLayout* layout = new QVBoxLayout(this);

    QHBoxLayout* input_type_layout = new QHBoxLayout();
    QLabel* input_label = new QLabel("User inputs:");
    QComboBox* input_type = new QComboBox();
    input_type->setModel(&input_type_model);
    input_type_layout->addWidget(input_label);
    input_type_layout->addWidget(input_type);
    input_type_layout->addStretch();
    layout->addLayout(input_type_layout);

    /* Foot pedal/head sensor configuration */

    QWidget* foot_pedal_details = new QWidget();
    QVBoxLayout* foot_pedal_layout = new QVBoxLayout(foot_pedal_details);

    this->pedals_available_mtms = new QComboBox();
    QHBoxLayout* pedal_mtms = new QHBoxLayout();
    pedal_mtms->addWidget(new QLabel("Which MTM controller are the foot pedals plugged into?"));
    pedal_mtms->addWidget(pedals_available_mtms);
    pedal_mtms->addStretch();
    foot_pedal_layout->addLayout(pedal_mtms);

    QHBoxLayout* head_sensor_layout = new QHBoxLayout();
    QComboBox* head_sensor_type = new QComboBox();
    head_sensor_type->setModel(&head_sensor_type_model);
    head_sensor_layout->addWidget(new QLabel("Do you have a head sensor to detect operator presence?"));
    head_sensor_layout->addWidget(head_sensor_type);
    head_sensor_layout->addStretch();
    foot_pedal_layout->addLayout(head_sensor_layout);

    this->head_sensor_available_mtms = new QComboBox();
    QStackedWidget* head_sensor_stack = new QStackedWidget();
    head_sensor_stack->addWidget(new QWidget());
    QWidget* head_sensor_mtms = new QWidget();
    QHBoxLayout* head_sensor_mtms_layout = new QHBoxLayout(head_sensor_mtms);
    head_sensor_mtms_layout->setMargin(0);
    head_sensor_mtms_layout->addWidget(new QLabel("Which MTM controller is your head sensor connected to?"));
    head_sensor_mtms_layout->addWidget(head_sensor_available_mtms);
    head_sensor_mtms_layout->addStretch();
    head_sensor_stack->addWidget(head_sensor_mtms);
    foot_pedal_layout->addWidget(head_sensor_stack);

    /* ForceDimension button inputs configuration */

    QWidget* forcedimension_input = new QWidget();
    QHBoxLayout* forcedimension_input_layout = new QHBoxLayout(forcedimension_input);
    this->available_forcedimensions = new QComboBox();
    forcedimension_input_layout->addWidget(new QLabel("Select which arm to get button events from:"));
    forcedimension_input_layout->addWidget(available_forcedimensions);
    forcedimension_input_layout->addStretch();

    QStackedWidget* user_input_stack = new QStackedWidget();
    user_input_stack->addWidget(new QWidget());
    user_input_stack->addWidget(foot_pedal_details);
    user_input_stack->addWidget(forcedimension_input);
    layout->addWidget(user_input_stack);

    QObject::connect(input_type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, user_input_stack](int index) {
        if (index < user_input_stack->count()) {
            user_input_stack->setCurrentIndex(index);
            this->model->type = static_cast<ConsoleInputType::Value>(index);
            emit this->model->updated();
        } else {
            user_input_stack->setCurrentIndex(0);
        }
    });

    QObject::connect(head_sensor_type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, head_sensor_stack](int index) {
        HeadSensorType::Value type = static_cast<HeadSensorType::Value>(index);
        this->model->head_sensor->type = type;
        emit this->model->updated();
        if (type == HeadSensorType::Value::DVRK || type == HeadSensorType::Value::ISI) {
            head_sensor_stack->setCurrentIndex(1);
        } else {
            head_sensor_stack->setCurrentIndex(0);
        }
    });

    QObject::connect(pedals_available_mtms, &QComboBox::currentTextChanged, this, [this](QString text) {
        std::string arm_name = text.toStdString();
        this->model->pedals->source_arm_name = arm_name;
        emit this->model->updated();
    });

    QObject::connect(head_sensor_available_mtms, &QComboBox::currentTextChanged, this, [this](QString text) {
        std::string arm_name = text.toStdString();
        this->model->head_sensor->source_arm_name = arm_name;
        emit this->model->updated();
    });

    QObject::connect(available_forcedimensions, &QComboBox::currentTextChanged, this, [this](QString text) {
        std::string arm_name = text.toStdString();
        this->model->force_dimension_buttons->source_arm_name = arm_name;
        emit this->model->updated();
    });

    updateAvailableArms();

    if (model.head_sensor != nullptr) {
        head_sensor_type->setCurrentIndex(model.head_sensor->type.id());
        head_sensor_available_mtms->setCurrentText(QString::fromStdString(model.head_sensor->source_arm_name));
    }

    if (model.type == ConsoleInputType::Value::FOOT_PEDALS) {
        input_type->setCurrentIndex(1);
        user_input_stack->setCurrentIndex(1);
        pedals_available_mtms->setCurrentText(QString::fromStdString(model.pedals->source_arm_name));
    } else if (model.type == ConsoleInputType::Value::FORCE_DIMENSION_BUTTONS) {
        input_type->setCurrentIndex(2);
        user_input_stack->setCurrentIndex(2);
        available_forcedimensions->setCurrentText(QString::fromStdString(model.force_dimension_buttons->source_arm_name));
    }
}

void ConsoleInputsEditor::updateAvailableArms() {
    pedals_available_mtms->clear();
    head_sensor_available_mtms->clear();
    available_forcedimensions->clear();

    for (int idx = 0; idx < arms->count(); idx++) {
        const ArmConfig& arm = arms->get(idx);
        bool native_or_simulated = arm.config_type == ArmConfigType::NATIVE || arm.config_type == ArmConfigType::SIMULATED;
        if (native_or_simulated && arm.type.isMTM()) {
            pedals_available_mtms->addItem(QString::fromStdString(arm.name));
            head_sensor_available_mtms->addItem(QString::fromStdString(arm.name));
        }

        if (arm.config_type == ArmConfigType::HAPTIC_MTM && arm.type.isMTM()) {
            available_forcedimensions->addItem(QString::fromStdString(arm.name));
        }
    }

    if (model->pedals != nullptr) {
        pedals_available_mtms->setCurrentText(QString::fromStdString(model->pedals->source_arm_name));
    }
    if (model->head_sensor != nullptr) {
        head_sensor_available_mtms->setCurrentText(QString::fromStdString(model->head_sensor->source_arm_name));
    }
    if (model->force_dimension_buttons != nullptr) {
        available_forcedimensions->setCurrentText(QString::fromStdString(model->force_dimension_buttons->source_arm_name));
    }
}

}

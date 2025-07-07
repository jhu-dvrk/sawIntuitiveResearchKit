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
#include <qboxlayout.h>
#include <qcombobox.h>
#include <qobject.h>
#include <qstackedwidget.h>
#include <qwidget.h>

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
    pedals_available_mtms->setPlaceholderText("No native MTMS available");
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
    head_sensor_available_mtms->setPlaceholderText("No native MTMS available");
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
    available_forcedimensions->setPlaceholderText("No haptic MTMS available");
    forcedimension_input_layout->addWidget(new QLabel("Select which arm to get button events from:"));
    forcedimension_input_layout->addWidget(available_forcedimensions);
    forcedimension_input_layout->addStretch();

    /* Custom component inputs configuration */

    QWidget* custom_inputs = new QWidget();
    QFormLayout* custom_inputs_layout = new QFormLayout(custom_inputs);
    QLabel* custom_input_label = new QLabel("Select component/interfaces that will provide user input button events:");
    custom_input_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    custom_inputs_layout->addRow(custom_input_label);
    custom_inputs_layout->addRow("Operator present button:", new QLineEdit("ForceDimensionSDK"));
    custom_inputs_layout->addRow("Clutch button:", new QLineEdit("ForceDimensionSDK"));
    custom_inputs_layout->addRow("Camera teleop button:", new QLineEdit("ForceDimensionSDK"));

    QStackedWidget* user_input_stack = new QStackedWidget();
    user_input_stack->addWidget(new QWidget());
    user_input_stack->addWidget(foot_pedal_details);
    user_input_stack->addWidget(forcedimension_input);
    user_input_stack->addWidget(custom_inputs);
    layout->addWidget(user_input_stack);

    QObject::connect(input_type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [user_input_stack](int index) {
        if (index < user_input_stack->count()) {
            user_input_stack->setCurrentIndex(index);
        } else {
            user_input_stack->setCurrentIndex(0);
        }
    });

    QObject::connect(head_sensor_type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [head_sensor_stack](int index) {
        HeadSensorType::Value type = static_cast<HeadSensorType::Value>(index);
        if (type == HeadSensorType::Value::DVRK || type == HeadSensorType::Value::ISI) {
            head_sensor_stack->setCurrentIndex(1);
        } else {
            head_sensor_stack->setCurrentIndex(0);
        }
    });

    updateAvailableArms();
}

void ConsoleInputsEditor::updateAvailableArms() {
    pedals_available_mtms->clear();
    head_sensor_available_mtms->clear();
    available_forcedimensions->clear();

    for (int idx = 0; idx < arms->count(); idx++) {
        const ArmConfig& arm = arms->get(idx);
        if (arm.config_type == ArmConfigType::NATIVE && arm.type.isMTM()) {
            pedals_available_mtms->addItem(QString::fromStdString(arm.name));
            head_sensor_available_mtms->addItem(QString::fromStdString(arm.name));
        }

        if (arm.config_type == ArmConfigType::HAPTIC_MTM && arm.type.isMTM()) {
            available_forcedimensions->addItem(QString::fromStdString(arm.name));
        }
    }
}

}

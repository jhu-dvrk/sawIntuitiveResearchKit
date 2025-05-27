/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-25

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "arm_editor.hpp"

namespace system_wizard {

ArmEditor::ArmEditor(ArmType type, QWidget* parent) : QFrame(parent) {
    setFrameStyle(QFrame::StyledPanel);
    setStyleSheet("system_wizard--ArmEditor { background-color: palette(base); border-radius: 10px; }");

    QFormLayout* form = new QFormLayout(this);

    QLineEdit* name_editor = new QLineEdit();
    name_editor->insert(QString::fromStdString(type.name()));

    type_selector = new QComboBox();
    type_selector->setModel(&type_model);
    type_selector->setCurrentIndex(static_cast<int>(type.idx()));

    QSpinBox* serial_selector = new QSpinBox();
    serial_selector->setRange(0, 999999);

    QCheckBox* skip_ros_bridge = new QCheckBox();
    QCheckBox* add_socket_server = new QCheckBox();

    QLineEdit* component_name = new QLineEdit();
    component_name->insert(QString::fromStdString(type.name()));

    QLineEdit* interface_name = new QLineEdit();
    interface_name->insert("Arm");

    QLineEdit* io_file_path = new QLineEdit();
    QLineEdit* pid_file_path = new QLineEdit();
    QLineEdit* arm_file_path = new QLineEdit();
    QLineEdit* kin_file_path = new QLineEdit();
    QLineEdit* base_frame = new QLineEdit();

    form->addRow("&Name: ", name_editor);
    form->addRow("&Type: ", type_selector);
    form->addRow("&Serial number: ", serial_selector);
    form->addRow("&Skip ROS bridge: ", skip_ros_bridge);
    form->addRow("&Add socket server: ", add_socket_server);
    form->addRow("&Component name: ", component_name);
    form->addRow("&Interface name: ", interface_name);
    form->addRow("&IO config file: ", io_file_path);
    form->addRow("&PID config file: ", pid_file_path);
    form->addRow("&Arm config file: ", arm_file_path);
    form->addRow("&Kinematic config file: ", kin_file_path);
    form->addRow("&Base frame transform: ", base_frame);
}

}

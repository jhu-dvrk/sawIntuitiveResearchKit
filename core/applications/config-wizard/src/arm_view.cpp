/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-27

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "arm_view.hpp"
#include "models/config_model.hpp"

namespace config_wizard {

ArmView::ArmView(SystemConfigModel& model, ListView& list_view, int id, QWidget* parent)
    : ItemView(list_view, id, parent), model(&model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(id);

    layout->addWidget(display);
    layout->addStretch();

    QPushButton* edit_button = new QPushButton("Edit");
    QObject::connect(edit_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.edit(this->id); });
    layout->addWidget(edit_button);

    QPushButton* delete_button = new QPushButton("Delete");
    QObject::connect(delete_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.try_delete(this->id); });
    layout->addWidget(delete_button);
}

void ArmView::updateData(int id) {
    this->id = id;

    const ArmConfig& arm = model->arm_configs->get(id);
    QString text = QString::fromStdString(arm.name);
    if (arm.config_type == ArmConfigType::ROS_ARM) {
        text += " (from ROS)";
    } else if (arm.config_type == ArmConfigType::HAPTIC_MTM) {
        text += " (haptic device)";
    } else if (arm.is_simulated.value_or(false)) {
        text += " (simulated)";
    }

    if (!arm.type.isSUJ()) {
        if (arm.base_frame.has_value()) {
            const BaseFrameConfig& bf = *arm.base_frame;
            std::string base_frame_name = bf.use_custom_transform ? bf.reference_frame_name : bf.base_frame_component.component_name;
            base_frame_name = base_frame_name.empty() ? "(no name)" : base_frame_name;
            text += ", base frame is " + QString::fromStdString(base_frame_name);
        } else if (arm.config_type == ArmConfigType::NATIVE || arm.config_type == ArmConfigType::SIMULATED) {
            text += ", no base frame";
        }
    }

    display->setText(text);
    display->setToolTip(QString::fromStdString(arm.type.acronym_expansion()));
}

}

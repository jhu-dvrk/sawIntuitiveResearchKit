/*
  Author(s):  Brendan Burkhart
  Created on: 2025-06-06

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "teleop_view.hpp"

#include <numeric>

namespace system_wizard {

PSMTeleopView::PSMTeleopView(ConsoleConfig& config, ListView& list_view, int idx, QWidget* parent)
    : ItemView(list_view, idx, parent), config(&config) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(idx);

    QPushButton* edit_button = new QPushButton("Edit");
    QPushButton* delete_button = new QPushButton("Delete");

    layout->addWidget(display);
    layout->addStretch();
    layout->addWidget(edit_button);
    layout->addWidget(delete_button);

    QObject::connect(edit_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.edit(this->id); });
    QObject::connect(delete_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.try_delete(this->id); });
}

void PSMTeleopView::updateData(int idx) {
    this->id = idx;

    const auto& teleop = config->psm_teleops->get(id);
    std::string arms = teleop.arm_names[0] + ", " + teleop.arm_names[1];
    QString text = QString::fromStdString(teleop.type.name() + ": " + arms);
    display->setText(text);
}

ECMTeleopView::ECMTeleopView(ConsoleConfig& config, ListView& list_view, int idx, QWidget* parent)
    : ItemView(list_view, idx, parent), config(&config) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(idx);

    QPushButton* edit_button = new QPushButton("Edit");
    QPushButton* delete_button = new QPushButton("Delete");

    layout->addWidget(display);
    layout->addStretch();
    layout->addWidget(edit_button);
    layout->addWidget(delete_button);

    QObject::connect(edit_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.edit(this->id); });
    QObject::connect(delete_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.try_delete(this->id); });
}

void ECMTeleopView::updateData(int idx) {
    this->id = idx;

    const auto& teleop = config->ecm_teleops->get(id);
    std::string arms = teleop.arm_names[0] + ", " + teleop.arm_names[1] + ", " + teleop.arm_names[2];
    QString text = QString::fromStdString(teleop.type.name() + ": " + arms);
    display->setText(text);
}

}

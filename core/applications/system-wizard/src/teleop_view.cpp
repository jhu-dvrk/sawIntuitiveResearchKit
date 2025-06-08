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

TeleopView::TeleopView(SystemConfigModel& model, ListView& list_view, int id, QWidget* parent)
    : ItemView(list_view, id, parent), model(&model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(id);

    QPushButton* edit_button = new QPushButton("Edit");
    QPushButton* delete_button = new QPushButton("Delete");

    layout->addWidget(display);
    layout->addStretch();
    layout->addWidget(edit_button);
    layout->addWidget(delete_button);

    QObject::connect(edit_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.edit(this->id); });
    QObject::connect(delete_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.try_delete(this->id); });
}

void TeleopView::updateData(int id) {
    this->id = id;

    const TeleopConfig& teleop = model->teleop_configs.get(id);
    std::string arms = std::accumulate(teleop.arms.begin(), teleop.arms.end(), std::string(""), [this](std::string acc, int arm_id) -> std::string {
        std::string arm_name = model->arm_configs.get(arm_id).name;
        return acc.empty() ? arm_name : acc + ", " + arm_name;
    });

    QString text = QString::fromStdString(teleop.type.name() + ": " + arms);
    display->setText(text);
}

TeleopViewFactory::TeleopViewFactory(SystemConfigModel& model) : model(&model) { }

TeleopView* TeleopViewFactory::create(int id, ListView& list_view) {
    return new TeleopView(*model, list_view, id);
}

}

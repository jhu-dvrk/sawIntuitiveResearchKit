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

namespace system_wizard {

ArmView::ArmView(SystemConfigModel* model, ListView& list_view, int id, QWidget* parent)
    : ItemView(list_view, id, parent), model(model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    const ArmConfig& arm = model->arms.at(id);
    QString text = QString::fromStdString(arm.description());

    display = new QLabel(text);
    QPushButton* edit_button = new QPushButton("Edit");
    QPushButton* delete_button = new QPushButton("Delete");

    layout->addWidget(display);
    layout->addStretch();
    layout->addWidget(edit_button);
    layout->addWidget(delete_button);

    QObject::connect(edit_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.edit(this->id); });
    QObject::connect(delete_button, &QPushButton::clicked, this, [this, &list_view](){ emit list_view.try_delete(this->id); });
}

void ArmView::updateData(int id) {
    this->id = id;

    const ArmConfig& arm = model->arms.at(id);
    QString text = QString::fromStdString(arm.description());
    display->setText(text);
}

ArmViewFactory::ArmViewFactory(SystemConfigModel* model) : model(model) { }

ArmView* ArmViewFactory::create(int id, ListView& list_view) {
    return new ArmView(model, list_view, id);
}

}

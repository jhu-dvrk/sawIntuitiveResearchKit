/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-28

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "io_view.hpp"

#include <cmath>

namespace system_wizard {

IOView::IOView(SystemConfigModel* model, ListView& list_view, int id, QWidget* parent)
    : ItemView(list_view, id, parent), model(model) {
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

void IOView::updateData(int id) {
    this->id = id;

    const IOConfig& io = model->io_configs.get(id);
    int frequency = int(std::round(1.0 / io.period_ms));
    QString text = QString::fromStdString(io.port.name()) + " @ " + QString::number(frequency);
    display->setText(text);
}

IOViewFactory::IOViewFactory(SystemConfigModel* model) : model(model) { }

IOView* IOViewFactory::create(int id, ListView& list_view) {
    return new IOView(model, list_view, id);
}

}

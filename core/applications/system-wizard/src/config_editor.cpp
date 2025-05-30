/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-17

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "config_editor.hpp"

#include "accordion.hpp"
#include "arm_view.hpp"

#include <QTreeWidget>

namespace system_wizard {

ConfigEditor::ConfigEditor(SystemConfigModel* model, ConfigSources* config_sources, QWidget* parent)
    : QWidget(parent), model(model), arm_editor(model, config_sources, this), io_editor(model, this) {
    QVBoxLayout* layout = new QVBoxLayout(this);

    QScrollArea* scroller = new QScrollArea();
    QWidget* scroller_contents = new QWidget();

    const QString lorem = "Lorem ipsum dolor sit amet consectetur adipiscing elit, sed duo eiusmod...";

    Accordion* ios = new Accordion("I/Os", "SteelBlue", this);
    io_factory = std::make_unique<IOViewFactory>(model);
    ListView* io_list = new ListView(&model->io_configs, io_factory.get());
    ios->setWidget(io_list);

    QObject::connect(io_list, &ListView::add, this, [this]() { io_editor.setId(-1); io_editor.open(); });
    QObject::connect(io_list, &ListView::edit, this, [this](int id) { io_editor.setId(id); io_editor.open(); });
    QObject::connect(io_list, &ListView::try_delete, &model->io_configs, &ListModelT<IOConfig>::deleteItem);

    auto default_io = IOConfig("io");
    model->io_configs.addItem(default_io);

    Accordion* arms = new Accordion("Arms", "LightSeaGreen", this);
    arm_factory = std::make_unique<ArmViewFactory>(model);
    ListView* arm_list = new ListView(&model->arm_configs, arm_factory.get());
    arms->setWidget(arm_list);

    QObject::connect(arm_list, &ListView::add, this, [this]() { arm_editor.open(); });
    QObject::connect(arm_list, &ListView::try_delete, &model->arm_configs, &ListModelT<ArmConfig>::deleteItem);

    Accordion* teleops = new Accordion("Teleops", "DodgerBlue", this);
    ListView* teleop_list = new ListView(&model->teleop_configs, arm_factory.get());
    teleops->setWidget(teleop_list);

    // QObject::connect(teleop_list, &ListView::add, &arm_editor, &ArmEditor::open);

    Accordion* consoles = new Accordion("Consoles", "Salmon", this);
    ListView* console_list = new ListView(&model->console_configs, arm_factory.get());
    consoles->setWidget(console_list);

    // QObject::connect(console_list, &ListView::add, &arm_editor, &ArmEditor::open);

    QVBoxLayout* scroller_layout = new QVBoxLayout(scroller_contents);
    scroller_layout->addWidget(ios);
    scroller_layout->addWidget(arms);
    scroller_layout->addWidget(teleops);
    scroller_layout->addWidget(consoles);
    scroller_layout->addStretch();
    scroller->setWidget(scroller_contents);
    scroller->setWidgetResizable(true);

    layout->addWidget(scroller);
}

void ConfigEditor::armsChanged() { 
    // TODO
}

}
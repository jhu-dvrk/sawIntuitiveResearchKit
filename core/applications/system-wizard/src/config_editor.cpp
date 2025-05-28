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

ConfigEditor::ConfigEditor(SystemConfigModel* model, QWidget* parent) : QWidget(parent), model(model), arm_editor(model, this) {
    QVBoxLayout* layout = new QVBoxLayout(this);

    QScrollArea* scroller = new QScrollArea();
    QWidget* scroller_contents = new QWidget();

    const QString lorem = "Lorem ipsum dolor sit amet consectetur adipiscing elit, sed duo eiusmod...";

    Accordion* ios = new Accordion("I/Os", "SteelBlue", this);
    io_factory = std::make_unique<IOViewFactory>(model);
    ListView* io_list = new ListView(arm_factory.get());
    ios->setWidget(io_list);

    auto test_io_config = IOConfig();
    test_io_config.period_ms = 1500.0;
    test_io_config.watchdog_timeout_ms = 10.0;
    model->addIO(test_io_config);

    QObject::connect(io_list, &ListView::try_delete, model, &SystemConfigModel::deleteIO);

    QObject::connect(model, &SystemConfigModel::ioAdded, io_list, &ListView::itemAdded);
    QObject::connect(model, &SystemConfigModel::ioUpdated, io_list, &ListView::itemUpdated);
    QObject::connect(model, &SystemConfigModel::ioDeleted, io_list, &ListView::itemRemoved);

    Accordion* arms = new Accordion("Arms", "LightSeaGreen", this);
    arm_factory = std::make_unique<ArmViewFactory>(model);
    ListView* arm_list = new ListView(arm_factory.get());
    arms->setWidget(arm_list);

    QObject::connect(arm_list, &ListView::add, &arm_editor, &ArmEditor::open);
    QObject::connect(arm_list, &ListView::try_delete, model, &SystemConfigModel::deleteArm);

    QObject::connect(model, &SystemConfigModel::armAdded, arm_list, &ListView::itemAdded);
    QObject::connect(model, &SystemConfigModel::armUpdated, arm_list, &ListView::itemUpdated);
    QObject::connect(model, &SystemConfigModel::armDeleted, arm_list, &ListView::itemRemoved);

    Accordion* teleops = new Accordion("Teleops", "DodgerBlue", this);
    ListView* teleop_list = new ListView(arm_factory.get());
    teleops->setWidget(teleop_list);

    // QObject::connect(teleop_list, &ListView::add, &arm_editor, &ArmEditor::open);

    Accordion* consoles = new Accordion("Consoles", "Salmon", this);
    ListView* console_list = new ListView(arm_factory.get());
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
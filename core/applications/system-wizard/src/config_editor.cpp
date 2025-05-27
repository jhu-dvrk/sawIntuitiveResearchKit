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
#include "arm_editor.hpp"
#include "io_editor.hpp"

#include <QTreeWidget>

namespace system_wizard {

ConfigEditor::ConfigEditor(QWidget* parent) : QWidget(parent) {
    QScrollArea* scroller = new QScrollArea();
    QWidget* scroller_contents = new QWidget();

    const QString lorem = "Lorem ipsum dolor sit amet consectetur adipiscing elit, sed duo eiusmod...";

    Accordion* ios = new Accordion("I/Os", "SteelBlue", this);
    IOEditor* io_editor = new IOEditor();
    ios->setContents(io_editor);

    Accordion* arms = new Accordion("Arms", "LightSeaGreen", this);
    QWidget* arm_list = new QWidget();
    QVBoxLayout* arm_list_layout = new QVBoxLayout(arm_list);
    arm_list_layout->addWidget(new ArmEditor(ArmType::Value::PSM));
    arm_list_layout->addWidget(new ArmEditor(ArmType::Value::MTM));
    arm_list_layout->addWidget(new ArmEditor(ArmType::Value::ECM));
    arms->setContents(arm_list);

    Accordion* teleops = new Accordion("Teleops", "DodgerBlue", this);
    QLabel* teleop_contents = new QLabel(lorem);
    teleops->setContents(teleop_contents);

    Accordion* consoles = new Accordion("Consoles", "Salmon", this);
    QLabel* console_contents = new QLabel(lorem);
    consoles->setContents(console_contents);

    QVBoxLayout* scroller_layout = new QVBoxLayout(scroller_contents);
    scroller_layout->addWidget(ios);
    scroller_layout->addWidget(arms);
    scroller_layout->addWidget(teleops);
    scroller_layout->addWidget(consoles);
    scroller_layout->addStretch();
    scroller->setWidget(scroller_contents);
    scroller->setWidgetResizable(true);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(scroller);
}

void ConfigEditor::armsChanged() { 
    // TODO
}

}
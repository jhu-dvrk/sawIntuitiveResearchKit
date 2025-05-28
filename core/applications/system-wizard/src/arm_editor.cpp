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

IntroPage::IntroPage(QWidget *parent) : QWizardPage(parent) {
    setTitle("Arm type");
    setSubTitle("Choose what type of arm to create");

    QLabel* test = new QLabel("aksjdlkajsdkjas", this);
}

ArmEditor::ArmEditor(SystemConfigModel* model, QWidget* parent) : QWizard(parent), model(model) {
    setPage(PAGE_INTRO, new IntroPage);

    setWindowTitle("Arm Editor");

    QObject::connect(button(QWizard::FinishButton), &QAbstractButton::clicked, this, &ArmEditor::done);

    setStartId(PAGE_INTRO);
}

void ArmEditor::done() {
    ArmConfig config = ArmConfig("Test", ArmType::Value::PSM_SOCKET);
    model->addArm(config);
}

}

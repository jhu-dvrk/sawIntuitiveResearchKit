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

#ifndef SYSTEM_WIZARD_ARM_EDITOR
#define SYSTEM_WIZARD_ARM_EDITOR

#include <QtWidgets>

#include "config_model.hpp"

namespace system_wizard {

class ArmEditor : public QWizard {
    Q_OBJECT

public:
    enum { PAGE_INTRO };

    ArmEditor(SystemConfigModel* model, QWidget* parent = nullptr);

private slots:
    //void showHelp();

private:
    void done();

    SystemConfigModel* model;
};

class IntroPage : public QWizardPage {
    Q_OBJECT

public:
    IntroPage(QWidget *parent = nullptr);

    int nextId() const override {
        return -1;
    }
};

}

#endif // SYSTEM_WIZARD_ARM_EDITOR

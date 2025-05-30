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
#include "config_sources.hpp"
#include "enum_list_model.hpp"
#include "list_view.hpp"

namespace system_wizard {

class ArmSourceViewFactory;

class ArmEditor : public QWizard {
    Q_OBJECT

public:
    enum { PAGE_QUICK_ARM, PAGE_BASIC };

    ArmEditor(SystemConfigModel* model, ConfigSources* config_sources, QWidget* parent = nullptr);

private:
    void done();

    SystemConfigModel* model;
    ConfigSources* config_sources;
};

class QuickArmPage : public QWizardPage {
    Q_OBJECT

public:
    QuickArmPage(ConfigSources* config_sources, QWidget *parent = nullptr);

    int nextId() const override {
        return ArmEditor::PAGE_BASIC;
    }

    void initializePage() override;

private:
    ConfigSources* config_sources;
    ListView* arm_list_view;

    std::unique_ptr<ItemViewFactory> factory;
};

class BasicArmPage : public QWizardPage {
    Q_OBJECT

public:
    BasicArmPage(QWidget *parent = nullptr);

    int nextId() const override {
        return -1;
    }

private:
    QLineEdit* name_input;

    QComboBox* type_selector;
    EnumListModel<ArmType> type_model;
};

}

#endif // SYSTEM_WIZARD_ARM_EDITOR

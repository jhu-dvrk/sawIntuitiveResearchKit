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

class ArmSourceView : public ItemView {
public:
    ArmSourceView(ListModelT<ConfigSources::Arm>& model, ListView& list_view, int id, QWidget* parent = nullptr);

    void updateData(int id) override;

private:
    ListModelT<ConfigSources::Arm>* model;
    QLabel* display;
};

class ArmSourceViewFactory : public ItemViewFactory {
public:
    ArmSourceViewFactory(ListModelT<ConfigSources::Arm>& model);

    ArmSourceView* create(int id, ListView& list_view);

private:
    ListModelT<ConfigSources::Arm>* model;
};

class ArmEditor : public QWizard {
    Q_OBJECT

public:
    enum { PAGE_QUICK_ARM, PAGE_BASIC };

    ArmEditor(SystemConfigModel& model, ConfigSources& config_sources, QWidget* parent = nullptr);

    void selectArmSource(ConfigSources::Arm arm_source);

private:
    void done();

    SystemConfigModel* model;
};

class QuickArmPage : public QWizardPage {
    Q_OBJECT

public:
    QuickArmPage(ArmEditor& editor, ConfigSources& config_sources, QWidget *parent = nullptr);

    int nextId() const override {
        return ArmEditor::PAGE_BASIC;
    }

    void initializePage() override;
    bool isComplete() const override;

private:
    ArmEditor* editor;
    ListView* arm_list_view;
    ArmSourceViewFactory arm_list_factory;
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

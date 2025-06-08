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

#ifndef SYSTEM_WIZARD_TELEOP_EDITOR
#define SYSTEM_WIZARD_TELEOP_EDITOR

#include <QtWidgets>

#include "config_model.hpp"
#include "config_sources.hpp"
#include "enum_list_model.hpp"
#include "list_model.hpp"
#include "list_view.hpp"

namespace system_wizard {

class TeleopOptionView : public ItemView {
public:
    TeleopOptionView(const ListModelT<ArmConfig>& arms, ListModelT<TeleopConfig>& model, ListView& list_view, int id, QWidget* parent = nullptr);

    void updateData(int id) override;

private:
    const ListModelT<ArmConfig>* arms;
    ListModelT<TeleopConfig>* model;
    QLabel* display;
};

class TeleopOptionViewFactory : public ItemViewFactory {
public:
    TeleopOptionViewFactory(const ListModelT<ArmConfig>& arms, ListModelT<TeleopConfig>& model);

    TeleopOptionView* create(int id, ListView& list_view);

private:
    const ListModelT<ArmConfig>* arms;
    ListModelT<TeleopConfig>* model;
};

class TeleopEditor : public QWizard {
    Q_OBJECT

public:
    enum { 
        PAGE_SUGGESTED_TELEOPS,
        PAGE_PSM_TELEOP,
        PAGE_ECM_TELEOP
    };

    TeleopEditor(SystemConfigModel& model, QWidget* parent = nullptr);

public slots:
    void setId(int id);

private:
    void done();

    QWizardPage* page;

    SystemConfigModel* model;
    TeleopConfig config;
    int id = -1;

    QLineEdit* name_input;
};

class SuggestedTeleopsPage : public QWizardPage {
    Q_OBJECT

public:
    SuggestedTeleopsPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent = nullptr);

    int nextId() const override { return next_page_id; }

    void initializePage() override;
    bool isComplete() const override;

private:
    int next_page_id = TeleopEditor::PAGE_SUGGESTED_TELEOPS;

    TeleopConfig* config;
    ListView* suggested_teleops_view;
    const ListModelT<ArmConfig>* arms;
    ListModelT<TeleopConfig> suggested_teleops;
    TeleopOptionViewFactory suggested_teleop_factory;
};

class PSMTeleopPage : public QWizardPage {
    Q_OBJECT

public:
    PSMTeleopPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent = nullptr);

    int nextId() const override { return -1; }

    void initializePage() override;

private:
    const ListModelT<ArmConfig>* arms;
    TeleopConfig* config;

    QComboBox* mtms;
    QComboBox* psms;
};

class ECMTeleopPage : public QWizardPage {
    Q_OBJECT

public:
    ECMTeleopPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent = nullptr);

    int nextId() const override { return -1; }

    void initializePage() override;

private:
    const ListModelT<ArmConfig>* arms;
    TeleopConfig* config;

    QComboBox* ecms;
    QComboBox* left_mtms;
    QComboBox* right_mtms;
};

}

#endif // SYSTEM_WIZARD_TELEOP_EDITOR

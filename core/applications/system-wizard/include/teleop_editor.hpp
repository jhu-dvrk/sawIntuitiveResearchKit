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

#include "list_view.hpp"

#include "models/config_model.hpp"
#include "models/list_model.hpp"

namespace system_wizard {

class PSMTeleopOptionView : public ItemView {
public:
    PSMTeleopOptionView(ListModelT<TeleopConfig>& model, ListView& list_view, int index, QWidget* parent = nullptr);

    void updateData(int index) override;

private:
    ListModelT<TeleopConfig>* model;
    QLabel* display;
};

class ECMTeleopOptionView : public ItemView {
public:
    ECMTeleopOptionView(ListModelT<TeleopConfig>& model, ListView& list_view, int index, QWidget* parent = nullptr);

    void updateData(int index) override;

private:
    ListModelT<TeleopConfig>* model;
    QLabel* display;
};

class SuggestedTeleopsPage : public QWizardPage {
    Q_OBJECT

public:
    SuggestedTeleopsPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent = nullptr);

    int nextId() const override { return next_page_id; }

    void setMode(bool psm) {
        psm_mode = psm;
    }

    void initializePage() override;
    bool isComplete() const override;

private:
    int next_page_id = -1;
    bool psm_mode = true;

    TeleopConfig* config;
    ListView* suggested_teleops_view;
    const ListModelT<ArmConfig>* arms;
    ListModelT<TeleopConfig> suggested_teleops;
};

class TeleopEditor : public QWizard {
    Q_OBJECT

public:
    enum {
        PAGE_SUGGESTED_TELEOPS
    };

    TeleopEditor(ConsoleConfig& console, const ListModelT<ArmConfig>& arms, QWidget* parent = nullptr);

public slots:
    void setId(bool psm, int id);

private:
    void save();

    SuggestedTeleopsPage* suggested_teleops_page;

    ConsoleConfig* console;
    TeleopConfig config;
    int index = -1;
    bool psm;

    QLineEdit* name_input;
};

}

#endif // SYSTEM_WIZARD_TELEOP_EDITOR

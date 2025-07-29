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

class TeleopOptionView : public ItemView {
public:
    TeleopOptionView(ListModelT<TeleopConfig>& model, ListView& list_view, int index, QWidget* parent = nullptr);

    void updateData(int index) override;

private:
    ListModelT<TeleopConfig>* model;
    QLabel* display;
};

class TeleopEditor : public QWizard {
    Q_OBJECT

public:
    enum {
        PAGE_SUGGESTED_TELEOPS,
        PAGE_TELEOP_PARAMETERS
    };

    TeleopEditor(ListModelT<TeleopConfig>& teleops, TeleopType type, const ListModelT<ArmConfig>& arms, QWidget* parent = nullptr);

public slots:
    void setId(int id);

private:
    void save();

    ListModelT<TeleopConfig>* teleops;
    TeleopType type;
    const ListModelT<ArmConfig>* arms;
    TeleopConfig config;

    int index = -1;
};

class SuggestedTeleopsPage : public QWizardPage {
    Q_OBJECT

public:
    SuggestedTeleopsPage(
        const ListModelT<ArmConfig>& available_arms,
        const ListModelT<TeleopConfig>& existing_teleops,
        TeleopConfig& config,
        QWidget *parent = nullptr
    );

    int nextId() const override { return TeleopEditor::PAGE_TELEOP_PARAMETERS; }

    void initializePage() override;
    bool isComplete() const override;

private:
    TeleopConfig* config;
    ListView* suggested_teleops_view;
    const ListModelT<ArmConfig>* arms;
    const ListModelT<TeleopConfig>* teleops;
    ListModelT<TeleopConfig> suggested_teleops;
};

class TeleopParametersPage : public QWizardPage {
    Q_OBJECT

public:
    TeleopParametersPage(
        TeleopConfig& config,
        const ListModelT<ArmConfig>& available_arms,
        QWidget *parent = nullptr
    );

    int nextId() const override { return -1; }

    void showEvent(QShowEvent *event) override;

private:
    bool usesHapticMTM() const;

    TeleopConfig* config;
    const ListModelT<ArmConfig>* arms;

    bool block_psm_base_frame_updates;

    QSlider* scale_selector;

    QWidget* haptic_mtm_details;
    QCheckBox* has_gripper;
    QCheckBox* has_actuated_wrist;

    QWidget* psm_base_frame_details;
    QComboBox* base_arms;
};

}

#endif // SYSTEM_WIZARD_TELEOP_EDITOR

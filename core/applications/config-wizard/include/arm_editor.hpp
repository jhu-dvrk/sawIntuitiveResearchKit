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

/**
 * Arm configuration editor, in either create-mode or edit-mode
 *
 * When creating a new arm, the editor appears as a multi-page
 * wizard. The pages are
 *   0: QuickArmPage - arms detected in config folder are available
 *                     to add as-is, or user to add a different type
 *   1: ArmTypePage - choose type, i.e. haptic MTM or ROS arm
 *   2: HapticMTMPage - for using Falcon/Omni/etc as an MTM
 *   3: ROSArmPage - for using MTM/PSM via ROS
 *   3: BaseFramePage - for editing native arm base frame config
 *
 *         start    -> QuickArmPage
 *  QuickArmPage    -> [ BaseFramePage, ArmTypePage ]
 *   ArmTypePage    -> [ HapticMTMPage, ROSArmPage, SimulatedArmPage ]
 * HapticMTMPage    -> end
 *    ROSArmPage    -> end
 * SimulatedArmPage -> BaseFramePage
 * BaseFramePage    -> end
 *
 * When editing a native arm later, user goes directly to BaseFramePage
 */

#ifndef SYSTEM_WIZARD_ARM_EDITOR
#define SYSTEM_WIZARD_ARM_EDITOR

#include <QtWidgets>

#include <memory>

#include "config_sources.hpp"
#include "file_selector.hpp"
#include "list_view.hpp"
#include "models/config_model.hpp"

namespace system_wizard {

class ArmSourceView : public ItemView {
public:
    ArmSourceView(ListModelT<ConfigSources::Arm>& model, ListView& list_view, int id, QWidget* parent = nullptr);

    void updateData(int id) override;

private:
    ListModelT<ConfigSources::Arm>* model;
    QLabel* display;
};

class ArmEditor : public QWizard {
    Q_OBJECT

public:
    enum {
        PAGE_QUICK_ARM,
        PAGE_ARM_TYPE,
        PAGE_HAPTIC_MTM,
        PAGE_ROS_ARM,
        PAGE_KIN_SIM,
        PAGE_BASE_FRAME
    };

    ArmEditor(SystemConfigModel& model, ConfigSources& config_sources, QWidget* parent = nullptr);

    void selectArmSource(ConfigSources::Arm arm_source);

public slots:
    void setId(int id);

private:
    void save();

    SystemConfigModel* model;
    ArmConfig config;
    int id = -1;
};

class QuickArmPage : public QWizardPage {
    Q_OBJECT

public:
    QuickArmPage(ArmConfig& config, const SystemConfigModel& model, ConfigSources& config_sources, QWidget *parent = nullptr);

    int nextId() const override { return next_page_id; }

    void initializePage() override;
    bool isComplete() const override;

private:
    int next_page_id = ArmEditor::PAGE_ARM_TYPE;

    ArmConfig* config;

    const SystemConfigModel* model;
    ConfigSources* config_sources;

    std::unique_ptr<ListModelT<ConfigSources::Arm>> available_arms;
    ListView* arm_list_view;
};

class ArmTypePage : public QWizardPage {
    Q_OBJECT

public:
    ArmTypePage(ArmConfig& config, QWidget *parent = nullptr);

    int nextId() const override { return next_page_id; }
    bool isComplete() const override { return false; }

private:
    ArmConfig* config;

    int next_page_id = ArmEditor::PAGE_ARM_TYPE;
};

class HapticMTMPage : public QWizardPage {
    Q_OBJECT

public:
    HapticMTMPage(ArmConfig& config, QWidget *parent = nullptr);

    int nextId() const override { return -1; }

    void initializePage() override;
    void showEvent(QShowEvent *event) override;
    bool isComplete() const override;

private:
    ArmConfig* config;

    QComboBox* haptic_device_selector;
    QStackedWidget* details;
    QComboBox* left_right_selector;
    QLineEdit* arm_name;
    FileSelector* config_selector;
};

class ROSArmPage : public QWizardPage {
    Q_OBJECT

public:
    ROSArmPage(ArmConfig& config, QWidget *parent = nullptr);

    int nextId() const override { return -1; }

    void showEvent(QShowEvent *event) override;
    bool isComplete() const override;

private:
    ArmConfig* config;

    QComboBox* arm_type;
    QLineEdit* arm_name;
    QDoubleSpinBox* period_input;
};

class SimulatedArmPage : public QWizardPage {
    Q_OBJECT

public:
    SimulatedArmPage(ArmConfig& config, QWidget *parent = nullptr);

    int nextId() const override { return next_page_id; }

    void showEvent(QShowEvent *event) override;
    bool isComplete() const override;

private:
    ArmConfig* config;
    int next_page_id = ArmEditor::PAGE_BASE_FRAME;

    QComboBox* arm_type;
    QLineEdit* arm_name;
    FileSelector* config_selector;
};

class BaseFramePage : public QWizardPage {
    Q_OBJECT

public:
    BaseFramePage(ArmConfig& config, SystemConfigModel& system_model, QWidget *parent = nullptr);

    int nextId() const override { return -1; }

    void initializePage() override;
    bool isComplete() const override;

private:
    ArmConfig* config;
    SystemConfigModel* system_model;

    QComboBox* base_frame_type;

    QSpinBox* ecm_mounting_pitch;
    QSpinBox* hrsv_pitch;
    QComboBox* suj_list;

    bool block_suj_updates;

    void updateBaseFrame();

    void setFrameToFixedECM();
    void setFrameToHRSV();
    void setFrameToHapticMTMUser();
    void setFrameToSetupJoints();
};

}

#endif // SYSTEM_WIZARD_ARM_EDITOR

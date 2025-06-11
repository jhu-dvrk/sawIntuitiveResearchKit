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
 * 0: QuickArmPage - arms detected in config folder are available
 *                   to add as-is, or user can create arm from scratch
 * 1: ArmTypePage - for arm-from-scratch, user indicates arm type, such
 *                  as haptic input device, arm-from-ROS, etc.
 * 2: HapticMTMPage - for using Falcon/Omni/etc as an MTM
 * 3: ROSArmPage - for using MTM/PSM via ROS
 * 4: NativeArmPage - for editing/creating a native arm
 *
 *     start -> 0
 *         0 -> [end, 1]
 *         1 -> [2, 3, 5]
 * {2, 3, 4} -> end
 *
 * When editing an arm later, user goes directly to pages 2-4
 * depending on what type of arm they are editing.
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
    enum { 
        PAGE_QUICK_ARM,
        PAGE_ARM_TYPE,
        PAGE_HAPTIC_MTM,
        PAGE_ROS_ARM
    };

    ArmEditor(SystemConfigModel& model, ConfigSources& config_sources, QWidget* parent = nullptr);

    void selectArmSource(ConfigSources::Arm arm_source);

public slots:
    void setId(int id);

private:
    void done();

    SystemConfigModel* model;
    ArmConfig config;
    int id = -1;
};

class QuickArmPage : public QWizardPage {
    Q_OBJECT

public:
    QuickArmPage(ArmConfig& config, SystemConfigModel& model, ConfigSources& config_sources, QWidget *parent = nullptr);

    int nextId() const override { return next_page_id; }

    void initializePage() override;
    bool isComplete() const override;

private:
    int next_page_id = ArmEditor::PAGE_ARM_TYPE;

    ArmConfig* config;

    SystemConfigModel* model;
    ConfigSources* config_sources;

    std::unique_ptr<ListModelT<ConfigSources::Arm>> available_arms;
    ListView* arm_list_view;
    ArmSourceViewFactory arm_list_factory;
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

private:
    ArmConfig* config;

    QComboBox* haptic_device_selector;
    QStackedWidget* details;
    QComboBox* left_right_selector;
};

class ROSArmPage : public QWizardPage {
    Q_OBJECT

public:
    ROSArmPage(ArmConfig& config, QWidget *parent = nullptr);
    
    int nextId() const override { return -1; }

    void initializePage() override;
    void showEvent(QShowEvent *event) override;

private:
    ArmConfig* config;

    QComboBox* arm_type;
    QLineEdit* arm_name;
};

// class NativeArmPage : public QWizardPage {
//     Q_OBJECT

// public:
//     NativeArmPage(ArmConfig& config, QWidget *parent = nullptr);

//     int nextId() const override {
//         return ArmEditor::PAGE_ARM_DETAILS;
//     }

//     void initializePage() override;

// private:
//     ArmConfig* config;

//     QCheckBox* add_socket_server;
//     QCheckBox* skip_ros_bridge;
// };

// class ArmDetailsPage : public QWizardPage {
//     Q_OBJECT

// public:
//     ArmDetailsPage(ArmConfig& config, QWidget *parent = nullptr);

//     int nextId() const override {
//         return -1;
//     }

//     void showEvent(QShowEvent *event) override;

//     void initializePage() override;

// private:
//     ArmConfig* config;

//     QLineEdit* name;
//     QComboBox* base_frame;
// };

}

#endif // SYSTEM_WIZARD_ARM_EDITOR

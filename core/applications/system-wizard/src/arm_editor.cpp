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

#include <cmath>

#include "cisstCommon/cmnConstants.h"
#include <cisstCommon/cmnPortability.h>
#include <qformlayout.h>
#include <qlabel.h>
#include <qline.h>
#include <qlineedit.h>
#include "cisstVector/vctTransformationTypes.h"

#include "models/config_model.hpp"
#include "sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h"

namespace system_wizard {

ArmSourceView::ArmSourceView(ListModelT<ConfigSources::Arm>& model, ListView& list_view, int id, QWidget* parent)
: ItemView(list_view, id, parent), model(&model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(id);

    layout->addWidget(display);
}

void ArmSourceView::updateData(int id) {
    ConfigSources::Arm arm = model->get(id);
    QString description;
    if (arm.type.isSUJ()) {
        description = QString::fromStdString(arm.name + " (" + arm.type.name() + ")");
    } else {
        description = QString::fromStdString(arm.name + "-" + arm.serial_number);
    }

    display->setText(description);
}

QuickArmPage::QuickArmPage(ArmConfig& config, const SystemConfigModel& model, ConfigSources& config_sources, QWidget *parent)
    : QWizardPage(parent),
      config(&config),
      model(&model),
      config_sources(&config_sources),
      available_arms(std::make_unique<ListModelT<ConfigSources::Arm>>()) {
    setTitle("Quick arm");
    setSubTitle("Choose from default arms");

    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel* source_label = new QLabel("Choose from available arms:");
    source_label->setWordWrap(true);
    layout->addWidget(source_label);

    auto arm_view_factory = [this](int index, ListView& list_view) {
        return std::make_unique<ArmSourceView>(*this->available_arms, list_view, index);
    };
    arm_list_view = new ListView(*available_arms, arm_view_factory, SelectionMode::SINGLE);
    arm_list_view->setEmptyMessage("No arms available - open a config folder");
    auto choose_arm = [this](int index) {
        ConfigSources::Arm source = available_arms->get(index);
        *this->config = ArmConfig(source.name, source.type, ArmConfigType::NATIVE);
        this->config->is_DQLA = source.is_DQLA;
        this->config->arm_file = source.config_file.filename().string();
        if (!source.serial_number.empty()) {
            this->config->serial_number = source.serial_number;
        }

        // fixed/Si SUJ do not need IO
        // only supply default IO if there is exactly one IO available, otherwise make user choose
        if ((source.type != ArmType::Value::SUJ_FIXED && source.type != ArmType::Value::SUJ_SI) && this->model->io_configs->count() == 1) {
            this->config->io_name = this->model->io_configs->get(0).name;
        }

        if (source.type.isSUJ()) {
            this->next_page_id = -1;
        } else {
            this->next_page_id = ArmEditor::PAGE_BASE_FRAME;
        }
        setFinalPage(this->next_page_id == -1);
    };
    QObject::connect(arm_list_view, &ListView::choose, this, [this, choose_arm](int index) {
        choose_arm(index);
        if (next_page_id == -1) {
            this->wizard()->accept();
        } else {
            this->wizard()->next();
        }
    });
    QObject::connect(arm_list_view, &ListView::selected, this, [this, choose_arm](int index, bool selected){
        if (selected) { choose_arm(index); }
        emit completeChanged();
    });

    layout->addWidget(arm_list_view);

    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);

    QLabel* custom_label = new QLabel("or add another type of arm, e.g. Falcon/Omni or arm from simulation");
    custom_label->setWordWrap(true);
    layout->addWidget(custom_label);
    QHBoxLayout* custom_arm_layout = new QHBoxLayout();
    QPushButton* custom_arm_button = new QPushButton("Configure arm");
    custom_arm_layout->addStretch();
    custom_arm_layout->addWidget(custom_arm_button);
    layout->addLayout(custom_arm_layout);

    QObject::connect(custom_arm_button, &QPushButton::clicked, this, [this]() {
        QWizard* wizard = this->wizard();
        if (wizard != nullptr) {
            next_page_id = ArmEditor::PAGE_ARM_TYPE;
            wizard->next();
        }
    });

    layout->addStretch();

    // prevent arm list from being stretched out after items are removed
    arm_list_view->layout()->setSizeConstraint(QLayout::SetMinimumSize);
}

void QuickArmPage::initializePage() {
    // Filter out already added arms
    {
        std::vector<ConfigSources::Arm> available;
        for (int i = 0; i < config_sources->getModel().count(); i++) {
            auto arm = config_sources->getModel().get(i);
            bool already_used = false;
            for (int j = 0; j < model->arm_configs->count(); j++) {
                if (model->arm_configs->get(j).name == arm.name) {
                    already_used = true;
                    break;
                }
            }

            if (!already_used) {
                available.push_back(arm);
            }
        }

        available_arms->replace(std::move(available));
    }

    arm_list_view->clearSelections();
    next_page_id = ArmEditor::PAGE_BASE_FRAME;

    QList<QWizard::WizardButton> button_layout;
    button_layout << QWizard::Stretch << QWizard::BackButton << QWizard::NextButton << QWizard::FinishButton << QWizard::CancelButton;
    wizard()->setButtonLayout(button_layout);

    // make sure dialog size is updated if arm source list has changed while hidden
    arm_list_view->updateGeometry();
}

bool QuickArmPage::isComplete() const {
    auto selections = arm_list_view->selectedItems();
    for (bool selected : selections) {
        if (selected) {
            return true;
        }
    }

    return false;
}

ArmTypePage::ArmTypePage(ArmConfig& config, QWidget *parent) : QWizardPage(parent), config(&config) {
    setTitle("Choose type of arm");

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel("What type of arm do you want to add?"));
    layout->addSpacing(10);
    QLabel* haptic_label = new QLabel("If you want to use a haptic input device such as a ForceDimension, Falcon, or Omni in place of an MTM arm:");
    haptic_label->setWordWrap(true);
    layout->addWidget(haptic_label);

    QPushButton* haptic_input = new QPushButton("Configure haptic input device");
    haptic_input->setFlat(true);
    haptic_input->setAutoFillBackground(true);
    QObject::connect(haptic_input, &QPushButton::clicked, this, [this]() {
        this->next_page_id = ArmEditor::PAGE_HAPTIC_MTM;
        this->wizard()->next();
    });
    layout->addWidget(haptic_input);
    layout->addSpacing(10);

    layout->addWidget(new QLabel("If you want to add a remote or simulated PSM/MTM arm available via ROS:"));
    QPushButton* via_ros = new QPushButton("Client arm for remote/simulated ROS arm");
    via_ros->setFlat(true);
    via_ros->setAutoFillBackground(true);
    QObject::connect(via_ros, &QPushButton::clicked, this, [this]() {
        this->next_page_id = ArmEditor::PAGE_ROS_ARM;
        this->wizard()->next();
    });
    layout->addWidget(via_ros);

    layout->addWidget(new QLabel("If you want to add a kinematic simulated arm:"));
    QPushButton* kin_simulated = new QPushButton("Kinematic simulated arm");
    kin_simulated->setFlat(true);
    kin_simulated->setAutoFillBackground(true);
    QObject::connect(kin_simulated, &QPushButton::clicked, this, [this]() {
        this->next_page_id = ArmEditor::PAGE_KIN_SIM;
        this->wizard()->next();
    });
    layout->addWidget(kin_simulated);
}

HapticMTMPage::HapticMTMPage(ArmConfig& config, QWidget *parent) : QWizardPage(parent), config(&config) {
    setTitle("Configure haptic device as MTM");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QFormLayout* device_type_form = new QFormLayout();

    // Haptic device selector determines which config options we display below,
    // as well as what shared library component we need to load
    haptic_device_selector = new QComboBox();
    haptic_device_selector->addItem("ForceDimension/Novint Falcon", 0);
    haptic_device_selector->addItem("Phantom Omni/Geomagic Touch",  1);
    device_type_form->addRow("Type of haptic input device:", haptic_device_selector);
    layout->addLayout(device_type_form);

    details = new QStackedWidget();

    QWidget* blank = new QWidget();
    details->addWidget(blank);

    // for sawForceDimensionSDK arms - Falcon and ForceDimension arms
    QWidget* force_dimension = new QWidget();
    QVBoxLayout* force_dimension_layout = new QVBoxLayout(force_dimension);
    QLabel* left_right_label = new QLabel("Do you want to use the haptic device as left or right input?");
    left_right_label->setWordWrap(true);
    left_right_selector = new QComboBox();
    left_right_selector->addItem("MTML (left input arm)",  0);
    left_right_selector->addItem("MTMR (right input arm)", 1);

    force_dimension_layout->addWidget(left_right_label);
    force_dimension_layout->addWidget(left_right_selector);

    QFormLayout* force_dimension_form = new QFormLayout();
    force_dimension_form->setMargin(0);

    arm_name = new QLineEdit();
    force_dimension_form->addRow("Arm name:", arm_name);
    QObject::connect(arm_name, &QLineEdit::textEdited, this, [this](const QString& text){
        this->config->name = text.toStdString();
        emit completeChanged();
    });

    auto configure_force_dimension = [this]() {
        // Configure ForceDimensionSDK component
        auto component = ComponentConfig();
        component.name = "ForceDimensionSDK";
        component.library_name = "sawForceDimensionSDK";
        component.class_name = "mtsForceDimension";
        if (config_selector->currentRelativeFile()) {
            component.configure_parameter = config_selector->currentRelativeFile().value();
        }

        // Configure arm's interface to component
        this->config->component = ComponentInterfaceConfig();
        this->config->component->component_name = component.name;
        this->config->component->interface_name = this->config->name;
        this->config->component->component = component;

        emit completeChanged();
    };

    config_selector = new FileSelector();
    std::filesystem::path share = sawIntuitiveResearchKit_SOURCE_CONFIG_DIR;
    config_selector->setReferenceDirectory(share);
    config_selector->setStartingDirectory(share);
    QObject::connect(config_selector, &FileSelector::selected, this, configure_force_dimension);
    force_dimension_form->addRow("Device config:", config_selector);
    force_dimension_layout->addLayout(force_dimension_form);

    QObject::connect(left_right_selector, QOverload<int>::of(&QComboBox::activated), this, [this, configure_force_dimension](int index) {
        std::filesystem::path share = sawIntuitiveResearchKit_SOURCE_CONFIG_DIR;

        if (index == 0) {
            this->config->name = "MTML";
            arm_name->setText("MTML");
            config_selector->setCurrentFile(share / "sawForceDimensionSDK-MTML.json");
        } else if (index == 1) {
            this->config->name = "MTMR";
            arm_name->setText("MTMR");
            config_selector->setCurrentFile(share / "sawForceDimensionSDK-MTMR.json");
        }

        configure_force_dimension();
    });

    details->addWidget(force_dimension);

    details->addWidget(new QLabel("Not supported yet - let us know if you want this feature!"));

    layout->addWidget(details);
    layout->addStretch();

    details->setCurrentIndex(0);
    QObject::connect(haptic_device_selector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        switch (index) {
        case 0:
            details->setCurrentIndex(1);
            this->config->haptic_device = index;
            break;
        default:
            details->setCurrentIndex(0);
            break;
        }

        emit completeChanged();
    });
}

void HapticMTMPage::initializePage() {
    haptic_device_selector->setCurrentIndex(-1);
    details->setCurrentIndex(0);
    left_right_selector->setCurrentIndex(-1);
    arm_name->setText("");
    config_selector->setCurrentFile("");
}

void HapticMTMPage::showEvent(QShowEvent *CMN_UNUSED(event)) {
    if (config->config_type != ArmConfigType::HAPTIC_MTM) {
        *config = ArmConfig("", ArmType::Value::MTM_GENERIC, ArmConfigType::HAPTIC_MTM);
    } else {
        config->type = ArmType::Value::MTM_GENERIC;
        if (config->haptic_device) {
            haptic_device_selector->setCurrentIndex(config->haptic_device.value());
        }

        // Should detect e.g. MTML2 as a left MTM
        std::string interface = config->component ? config->component->interface_name : "";
        if (interface.size() >= 4 && interface.substr(0, 4) == "MTML") {
            left_right_selector->setCurrentIndex(0);
        } else if (interface.size() >= 4 && interface.substr(0, 4) == "MTMR") {
            left_right_selector->setCurrentIndex(1);
        }

        arm_name->setText(QString::fromStdString(config->name));

        if (config->component && config->component->component) {
            auto comp = config->component->component;
            config_selector->setCurrentFile(comp->configure_parameter);
        }
    }
}

bool HapticMTMPage::isComplete() const {
    bool device_selected = haptic_device_selector->currentIndex() != -1;
    bool device_configured = left_right_selector->currentIndex() != -1 && !arm_name->text().isEmpty();

    return device_selected && device_configured && config_selector->currentFile().has_value();
}

ROSArmPage::ROSArmPage(ArmConfig& config, QWidget *parent) : QWizardPage(parent), config(&config) {
    setTitle("Remote arm via ROS");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* description1 = new QLabel("Client arm for a remote dVRK arm available via ROS, either an actual arm or a simulateion provided by e.g. the AMBF simulator.");
    description1->setWordWrap(true);
    layout->addWidget(description1);
    QLabel* description2 = new QLabel("Client arm name (e.g. \"PSM1\") must match ROS namespace used for remote arm");
    description2->setWordWrap(true);
    layout->addWidget(description2);

    QFormLayout* form = new QFormLayout();

    auto update_component = [this]() {
        // Configure dvrk_arm_from_ros component
        auto component = ComponentConfig();
        component.name = this->config->name;
        if (period_input->value() > 0.0) {
            component.period = period_input->value();
        } else {
            component.period = 0.01; // 100 Hz default
        }
        component.library_name = "dvrk_arm_from_ros";
        if (this->config->type.isPSM()) {
          component.class_name = "dvrk_psm_from_ros";
        } else if (this->config->type.isMTM()) {
            component.class_name = "dvrk_mtm_from_ros";
        } else {
            component.class_name = "dvrk_arm_from_ros";
        }

        // Configure arm's interface to component
        this->config->component = ComponentInterfaceConfig();
        this->config->component->component_name = this->config->name;
        this->config->component->interface_name = this->config->name;
        this->config->component->component = component;
    };

    arm_type = new QComboBox();
    arm_type->addItem("PSM", 0);
    arm_type->addItem("MTM", 1);
    QObject::connect(arm_type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, update_component](int index) {
        if (index == 0) {
            this->config->type = ArmType::Value::PSM_GENERIC;
            this->config->config_type = ArmConfigType::ROS_ARM;
        } else if (index == 1) {
            this->config->type = ArmType::Value::MTM_GENERIC;
            this->config->config_type = ArmConfigType::ROS_ARM;
        }

        update_component();
        emit completeChanged();
    });

    form->addRow("Arm type:", arm_type);

    arm_name = new QLineEdit();
    form->addRow("Arm name:", arm_name);
    QObject::connect(arm_name, &QLineEdit::textChanged, this, [this, update_component](const QString& text){
        this->config->name = text.toStdString();
        update_component();
        emit completeChanged();
    });

    period_input = new QDoubleSpinBox();
    period_input->setRange(0.000, 0.050);
    period_input->setSingleStep(0.001);
    period_input->setSpecialValueText("Default (0.01 s/100 Hz)");
    QObject::connect(period_input, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, update_component);

    QLabel* period_label = new QLabel("Period (seconds):");
    period_label->setToolTip("Period that ROS client component will run at");
    form->addRow(period_label, period_input);

    layout->addLayout(form);
}

void ROSArmPage::showEvent(QShowEvent *CMN_UNUSED(event)) {
    if (config->config_type != ArmConfigType::ROS_ARM) {
        *config = ArmConfig("", static_cast<ArmType::Value>(-1), ArmConfigType::ROS_ARM);
        arm_type->setCurrentIndex(-1);
        arm_name->setText("");
    } else {
        if (config->type == ArmType(ArmType::Value::PSM_GENERIC)) {
            arm_type->setCurrentIndex(0);
        } else if (config->type == ArmType(ArmType::Value::MTM_GENERIC)) {
            arm_type->setCurrentIndex(1);
        } else {
            arm_type->setCurrentIndex(-1);
        }
        if (config->component && config->component->component) {
            const auto comp = config->component->component;
            if (comp->period.has_value()) {
                period_input->setValue(comp->period.value());
            }
        }
        arm_name->setText(QString::fromStdString(config->name));
    }
}

bool ROSArmPage::isComplete() const {
    return !arm_name->text().isEmpty() && arm_type->currentIndex() != -1;
}

SimulatedArmPage::SimulatedArmPage(ArmConfig& config, QWidget *parent) : QWizardPage(parent), config(&config) {
    setTitle("Kinematic arm simulation");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* description1 = new QLabel("Simple kinematic (no dynamics/collision/contact) simulation of a dVRK arm.");
    description1->setWordWrap(true);
    layout->addWidget(description1);
    QLabel* description2 = new QLabel("This is for the simple built-in dVRK simulation - if you want to use an external simulator (e.g. AMBF or Isaac Sim), please instead use a ROS arm and have the simulator communicate with the dVRK via ROS.");
    description2->setWordWrap(true);
    layout->addWidget(description2);

    QFormLayout* form = new QFormLayout();

    arm_type = new QComboBox();
    arm_type->addItem("PSM", 0);
    arm_type->addItem("MTM", 1);
    arm_type->addItem("ECM", 2);
    arm_type->addItem("SUJ", 3);
    QObject::connect(arm_type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (index == 0) {
            this->config->type = ArmType::Value::PSM;
        } else if (index == 1) {
            this->config->type = ArmType::Value::MTM;
        } else if (index == 2) {
            this->config->type = ArmType::Value::ECM;
        } else if (index == 3) {
            this->config->type = ArmType::Value::SUJ_CLASSIC;
        }
        next_page_id = this->config->type.isSUJ() ? -1 : ArmEditor::PAGE_BASE_FRAME;
        setFinalPage(next_page_id == -1);

        emit completeChanged();
    });
    form->addRow("Arm type:", arm_type);

    arm_name = new QLineEdit();
    form->addRow("Arm name:", arm_name);
    QObject::connect(arm_name, &QLineEdit::textChanged, this, [this](const QString& text){
        this->config->name = text.toStdString();
        emit completeChanged();
    });

    config_selector = new FileSelector();
    std::filesystem::path share = sawIntuitiveResearchKit_SOURCE_CONFIG_DIR;
    config_selector->setStartingDirectory(share / "arm");
    config_selector->setReferenceDirectory(share);
    QObject::connect(config_selector, &FileSelector::selected, this, [this](std::filesystem::path file) {
        this->config->arm_file = file;
        emit completeChanged();
    });
    form->addRow("Kinematic config:", config_selector);

    layout->addLayout(form);
}

void SimulatedArmPage::showEvent(QShowEvent *CMN_UNUSED(event)) {
    if (config->config_type != ArmConfigType::SIMULATED) {
        *config = ArmConfig("", static_cast<ArmType::Value>(-1), ArmConfigType::SIMULATED);
        arm_type->setCurrentIndex(-1);
        arm_name->setText("");
    } else {
        if (config->type == ArmType(ArmType::Value::PSM)) {
            arm_type->setCurrentIndex(0);
        } else if (config->type == ArmType(ArmType::Value::MTM)) {
            arm_type->setCurrentIndex(1);
        } else if (config->type == ArmType(ArmType::Value::ECM)) {
            arm_type->setCurrentIndex(2);
        } else if (config->type == ArmType(ArmType::Value::SUJ_CLASSIC)) {
            arm_type->setCurrentIndex(3);
        } else {
            arm_type->setCurrentIndex(-1);
        }

        next_page_id = this->config->type.isSUJ() ? -1 : ArmEditor::PAGE_BASE_FRAME;
        bool is_last_page = next_page_id == -1;
        // prevent infinite recursion
        if (isFinalPage() != is_last_page) {
            setFinalPage(is_last_page);
        }

        arm_name->setText(QString::fromStdString(config->name));
        if (config->arm_file.has_value()) {
            config_selector->setCurrentFile(config->arm_file.value());
        }
    }

    config->is_simulated = true;
}

bool SimulatedArmPage::isComplete() const {
    return !arm_name->text().isEmpty() && arm_type->currentIndex() != -1;
}

BaseFramePage::BaseFramePage(ArmConfig& config, SystemConfigModel& system_model, QWidget *parent) :
    QWizardPage(parent), config(&config), system_model(&system_model)
{
    setTitle("Arm base frame");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* description1 = new QLabel("Base frame transform configuration for native dVRK arms");
    description1->setWordWrap(true);
    layout->addWidget(description1);
    QLabel* description2 = new QLabel("Other arms, such as arms-from-ROS or haptic device MTMs, do not support base frames");
    description2->setWordWrap(true);
    layout->addWidget(description2);
    QLabel* description3 = new QLabel("For optimal teleop behavior using an ECM and stereo display, the PSM's base frame should match the ECM, and the MTM should match the stereo display.\nIf teleop is done without a camera, then the PSM and MTM base frames should have the same orientation in the real world frame.");
    description3->setWordWrap(true);
    layout->addWidget(description3);

    QFormLayout* form = new QFormLayout();

    base_frame_type = new QComboBox();
    base_frame_type->addItem("None", 0);
    base_frame_type->addItem("ForceDimension device user", 1);
    base_frame_type->addItem("Fixed ECM", 2);
    base_frame_type->addItem("HRSV (Surgeon console stereo display)", 3);
    base_frame_type->addItem("Setup Joints", 4);

    QStackedWidget* details = new QStackedWidget();
    details->addWidget(new QLabel()); // no details for "None" frame
    details->addWidget(new QLabel("Re-orients a PSM to match ForceDimension convention for teleoperation (without a stereo camera/display)"));

    QWidget* ecm_mounting_details = new QWidget();
    QFormLayout* ecm_mounting_layout = new QFormLayout(ecm_mounting_details);
    ecm_mounting_pitch = new QSpinBox();
    ecm_mounting_pitch->setRange(-90, 0);
    ecm_mounting_pitch->setSingleStep(5);
    ecm_mounting_pitch->setValue(-45);
    QLabel* fixed_ecm_description = new QLabel("Re-orients PSM base frame to match a fixed camera frame. Camera is assumed to have X-axis parallel to PSM X-axis, mounting pitch angle is then 0 degrees when camera is parallel to ground and -90 degrees have camera is pointing straight down.");
    fixed_ecm_description->setWordWrap(true);
    ecm_mounting_layout->addRow(fixed_ecm_description);
    ecm_mounting_layout->addRow("Camera mounting pitch (degrees):", ecm_mounting_pitch);
    details->addWidget(ecm_mounting_details);

    QWidget* hrsv_mounting_details = new QWidget();
    QFormLayout* hrsv_mounting_layout = new QFormLayout(hrsv_mounting_details);
    hrsv_pitch = new QSpinBox();
    hrsv_pitch->setRange(-90, 0);
    hrsv_pitch->setSingleStep(5);
    hrsv_pitch->setValue(-45);
    QLabel* hrsv_description = new QLabel("Re-orients MTM base frame to match a stereo display. Pitch is 0 degrees when stereo display depth axis is parallel to ground, and -90 when pointing at ground. Typically mounting pitch is -30 or -45 degrees.");
    hrsv_description->setWordWrap(true);
    hrsv_mounting_layout->addRow(hrsv_description);
    hrsv_mounting_layout->addRow("Stereo display pitch (degrees):", hrsv_pitch);
    details->addWidget(hrsv_mounting_details);

    QWidget* suj_details = new QWidget();
    QFormLayout* suj_layout = new QFormLayout(suj_details);
    suj_list = new QComboBox();
    QLabel* suj_description = new QLabel("For PSMs and ECMs mounted on passive setup joints, or fixed virtual setup joints determined by e.g. hand-eye registration");
    suj_description->setWordWrap(true);
    suj_layout->addRow(suj_description);
    suj_layout->addRow("Setup Joints: ", suj_list);
    details->addWidget(suj_details);

    QObject::connect(ecm_mounting_pitch, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int){ updateBaseFrame(); });
    QObject::connect(hrsv_pitch, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int){ updateBaseFrame(); });
    QObject::connect(suj_list, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int){
        if (block_suj_updates) { return; }
        updateBaseFrame();
        completeChanged();
    });
    QObject::connect(base_frame_type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, details](int index) {
        details->setCurrentIndex(index);
        updateBaseFrame();
        completeChanged();
    });

    form->addRow("Base frame:", base_frame_type);
    layout->addLayout(form);
    layout->addWidget(details);
}

void BaseFramePage::initializePage() {
    // temporarily block handling changes to selected SUJ, otherwise the
    // the suj_list->addItem(...) calls will make suj_list emit currentIndexChanged,
    // and the config's SUJ will be over-written with last SUJ to be added
    block_suj_updates = true;

    suj_list->clear();
    for (int i = 0; i < system_model->arm_configs->count(); i++) {
        auto arm = system_model->arm_configs->get(i);
        if (arm.type.isSUJ()) {
            // We use arm.type.name() instead of arm.name since all SUJ components
            // are currently named "SUJ"
            suj_list->addItem(QString::fromStdString(arm.type.name()));
        }
    }

    block_suj_updates = false;

    if (config->base_frame.has_value()) {
        if (!config->base_frame->use_custom_transform) {
            std::string suj_name = config->base_frame->base_frame_component.component_name;
            base_frame_type->setCurrentIndex(4);
            suj_list->setCurrentText(QString::fromStdString(suj_name));
        } else if (config->base_frame->reference_frame_name == "user") {
            base_frame_type->setCurrentIndex(1);
        } else if (config->base_frame->reference_frame_name == "ECM") {
            // rotation of Y-Axis around X-axis
            double theta = std::atan2(config->base_frame->transform.at(2, 1), config->base_frame->transform.at(1, 1));
            double pitch = theta + cmnPI_2;
            base_frame_type->setCurrentIndex(2);
            ecm_mounting_pitch->setValue(std::round(pitch * cmn180_PI));
        } else if (config->base_frame->reference_frame_name == "HRSV") {
            // rotation of Y-Axis around X-axis (but negative to rotation for flip about Y axis)
            double theta = -std::atan2(config->base_frame->transform.at(2, 1), config->base_frame->transform.at(1, 1));
            base_frame_type->setCurrentIndex(3);
            hrsv_pitch->setValue(std::round(theta * cmn180_PI));
        }
    } else {
        base_frame_type->setCurrentIndex(0);
    }
}

bool BaseFramePage::isComplete() const {
    return base_frame_type->currentIndex() != 4 || suj_list->currentIndex() != -1;
}

void BaseFramePage::updateBaseFrame() {
    int frame_type = base_frame_type->currentIndex();
    if (frame_type == 0) {
        this->config->base_frame = {};
    } else if (frame_type == 1) {
        setFrameToHapticMTMUser();
    } else if (frame_type == 2) {
        setFrameToFixedECM();
    } else if (frame_type == 3) {
        setFrameToHRSV();
    } else if (frame_type == 4) {
        setFrameToSetupJoints();
    }
}

void BaseFramePage::setFrameToFixedECM() {
    this->config->base_frame = BaseFrameConfig();
    this->config->base_frame->use_custom_transform = true;
    this->config->base_frame->reference_frame_name = "ECM";
    double mounting_pitch = cmnPI_180 * ecm_mounting_pitch->value(); // convert radians to degrees
    double theta = mounting_pitch - cmnPI_2; // convert from pitch relative to horizontal to angle relative to PSM default base frame
    this->config->base_frame->transform = vctFrm4x4(vctAxAnRot3(vct3(1.0, 0.0, 0.0), theta), vct3(0.0));
}

void BaseFramePage::setFrameToHRSV() {
    this->config->base_frame = BaseFrameConfig();
    this->config->base_frame->use_custom_transform = true;
    this->config->base_frame->reference_frame_name = "HRSV";
    double mounting_pitch = cmnPI_180 * hrsv_pitch->value(); // convert radians to degrees
    // Rotate by display mounting pitch, then do 180 degree rotation to flip X-axis direction to match ECM/stereo display
    vctRot3 rotation = vctRot3(vctAxAnRot3(vct3(0.0, 1.0, 0.0), cmnPI)) * vctRot3(vctAxAnRot3(vct3(1.0, 0.0, 0.0), mounting_pitch));
    this->config->base_frame->transform = vctFrm4x4(rotation, vct3(0.0, 0.400, 0.475));
}

void BaseFramePage::setFrameToHapticMTMUser() {
    this->config->base_frame = BaseFrameConfig();
    this->config->base_frame->use_custom_transform = true;
    this->config->base_frame->reference_frame_name = "user";
    // -90 degree rotation about +Z axis
    this->config->base_frame->transform = vctFrm4x4(vctAxAnRot3(vct3(0.0, 0.0, 1.0), -cmnPI_2), vct3(0.0));
}

void BaseFramePage::setFrameToSetupJoints() {
    this->config->base_frame = BaseFrameConfig();
    this->config->base_frame->use_custom_transform = false;
    this->config->base_frame->base_frame_component = ComponentInterfaceConfig();
    // dvrk_system currently assumes name is always "SUJ"
    std::string suj_name = "SUJ"; // suj_list->currentText().toStdString();
    this->config->base_frame->base_frame_component.component_name = suj_name;
    this->config->base_frame->base_frame_component.interface_name = this->config->name;
}

ArmEditor::ArmEditor(SystemConfigModel& model, ConfigSources& config_sources, QWidget* parent)
    : QWizard(parent), model(&model), config("Arm", ArmType::Value::PSM, ArmConfigType::NATIVE) {
    setPage(PAGE_QUICK_ARM, new QuickArmPage(config, model, config_sources));
    setPage(PAGE_ARM_TYPE, new ArmTypePage(config));
    setPage(PAGE_HAPTIC_MTM, new HapticMTMPage(config));
    setPage(PAGE_ROS_ARM, new ROSArmPage(config));
    setPage(PAGE_KIN_SIM, new SimulatedArmPage(config));
    setPage(PAGE_BASE_FRAME, new BaseFramePage(config, model));

    setWizardStyle(QWizard::ModernStyle);
    setOptions(QWizard::NoBackButtonOnStartPage);
    setWindowTitle("Arm Config Editor");

    setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);

    setStartId(PAGE_QUICK_ARM);

    QObject::connect(this, &QDialog::accepted, this, &ArmEditor::save);
}

void ArmEditor::save() {
    if (id < 0) {
        model->arm_configs->appendItem(config);
    } else {
        model->arm_configs->ref(id) = config;
        model->arm_configs->updateItem(id);
    }
}

void ArmEditor::setId(int id) {
    this->id = id;

    if (id >= 0) {
        config = model->arm_configs->get(id);
        setWindowTitle("Configure arm: " + QString::fromStdString(config.name));
        if (config.config_type == ArmConfigType::HAPTIC_MTM) {
            setStartId(PAGE_HAPTIC_MTM);
        } else if (config.config_type == ArmConfigType::ROS_ARM) {
            setStartId(PAGE_ROS_ARM);
        }  else if (config.config_type == ArmConfigType::SIMULATED) {
            setStartId(PAGE_KIN_SIM);
        } else {
            setStartId(PAGE_BASE_FRAME);
        }
    } else {
        config = ArmConfig("", static_cast<ArmType::Value>(-1), static_cast<ArmConfigType>(-1));
        setStartId(PAGE_QUICK_ARM);
        setWindowTitle("Configure new arm");
    }

    // Hack to make QWizard reset properly when it is opened
    restart();
}

}

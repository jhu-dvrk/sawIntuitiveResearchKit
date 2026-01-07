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

#include "teleop_editor.hpp"

#include "models/config_model.hpp"
#include <iomanip>
#include <ios>
#include <qboxlayout.h>
#include <qcheckbox.h>
#include <sstream>

namespace config_wizard {

TeleopOptionView::TeleopOptionView(ListModelT<TeleopConfig>& model, ListView& list_view, int index, QWidget* parent)
    : ItemView(list_view, index, parent), model(&model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(index);

    layout->addWidget(display);
}

void TeleopOptionView::updateData(int index) {
    const auto& teleop = model->get(index);

    std::string arms;
    for (size_t idx = 0; idx < teleop.arm_names.size(); idx++) {
        if (idx > 0) { arms += ", "; }
        arms += teleop.arm_names[idx];
    }

    QString text = QString::fromStdString(teleop.type.name() + ": " + arms);
    display->setText(text);
}

TeleopEditor::TeleopEditor(ListModelT<TeleopConfig>& teleops, TeleopType type, const ListModelT<ArmConfig>& arms, QWidget* parent)
    : QWizard(parent), teleops(&teleops), type(type), arms(&arms), config("Teleop", type) {
    SuggestedTeleopsPage* suggested_teleops_page = new SuggestedTeleopsPage(arms, teleops, config);
    TeleopParametersPage* teleop_parameters_page = new TeleopParametersPage(config, arms);

    setPage(PAGE_SUGGESTED_TELEOPS, suggested_teleops_page);
    setPage(PAGE_TELEOP_PARAMETERS, teleop_parameters_page);

    setWizardStyle(QWizard::ModernStyle);
    setOption(QWizard::NoBackButtonOnStartPage);
    setWindowTitle(QString::fromStdString(type.name()) + " config editor");

    setStartId(PAGE_SUGGESTED_TELEOPS);

    setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);

    QObject::connect(this, &QDialog::accepted, this, &TeleopEditor::save);
}

void TeleopEditor::setId(int index) {
    this->index = index;

    if (index >= 0) {
        config = teleops->get(index);
        setStartId(PAGE_TELEOP_PARAMETERS);
        setWindowTitle("Configure teleop: " + QString::fromStdString(config.name));
    } else {
        config = TeleopConfig("", type);
        config.provideSources(*arms);
        setStartId(PAGE_SUGGESTED_TELEOPS);
        setWindowTitle("Configure new teleop");
    }

    // Hack to make QWizard reset properly when it is opened
    restart();
}

void TeleopEditor::save() {
    if (index < 0) {
        teleops->appendItem(config);
    } else {
        teleops->ref(index) = config;
        teleops->updateItem(index);
    }
}

SuggestedTeleopsPage::SuggestedTeleopsPage(
    const ListModelT<ArmConfig>& arms,
    const ListModelT<TeleopConfig>& teleops,
    TeleopConfig& config, QWidget *parent)
    : QWizardPage(parent), config(&config), arms(&arms), teleops(&teleops) {
    setTitle("Suggested teleops");
    setSubTitle("Suggested teleops are based on available arms");

    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel* source_label = new QLabel("Choose from suggested teleops:");
    source_label->setWordWrap(true);
    layout->addWidget(source_label);

    auto suggested_teleop_factory = [this](int index, ListView& view) -> std::unique_ptr<ItemView> {
        return std::make_unique<TeleopOptionView>(suggested_teleops, view, index);
    };
    suggested_teleops_view = new ListView(suggested_teleops, suggested_teleop_factory, SelectionMode::SINGLE);
    suggested_teleops_view->setEmptyMessage("No more suggestions available");
    QObject::connect(suggested_teleops_view, &ListView::choose, this, [this, &model=suggested_teleops](int index){
        *this->config = model.get(index);
        this->wizard()->next();
    });
    QObject::connect(suggested_teleops_view, &ListView::selected, this, [this, &model=suggested_teleops](int index, bool selected){
        if (selected) {
            *this->config = model.get(index);
        }
    });
    QObject::connect(suggested_teleops_view, &ListView::selected, this, &QWizardPage::completeChanged);

    layout->addWidget(suggested_teleops_view);

    layout->addStretch();

    // prevent arm list from being stretched out after items are removed
    suggested_teleops_view->layout()->setSizeConstraint(QLayout::SetMinimumSize);
}

void SuggestedTeleopsPage::initializePage() {
    suggested_teleops_view->clearSelections();
    suggested_teleops.clear();

    int n = arms->count();

    auto add_candidate = [this](TeleopConfig candidate) {
        // check if candidate is already added to the console
        for (int j = 0; j < teleops->count(); j++) {
            if (teleops->get(j).arm_names == candidate.arm_names) {
                return;
            }
        }

        // candidate isn't used yet, add it to the suggested list
        candidate.provideSources(*arms);
        suggested_teleops.appendItem(candidate);
    };

    if (config->type == TeleopType::Value::PSM_TELEOP) {
        // PSM teleops need one PSM, one MTM
        for (int i = 0; i < n; i++) {
            const ArmConfig& mtm = arms->get(i);
            if (!mtm.type.isMTM()) { continue; }

            for (int j = 0; j < n; j++) {
                const ArmConfig& psm = arms->get(j);
                if (!psm.type.isPSM()) { continue; }

                std::string name = mtm.name + "-" + psm.name + " Teleop";
                TeleopConfig teleop = TeleopConfig(name, TeleopType::Value::PSM_TELEOP);
                teleop.arm_names = { mtm.name, psm.name };

                add_candidate(teleop);
            }
        }
    } else {
        // ECM teleops need one ECM, two MTMs
        for (int i = 0; i < n; i++) {
            const ArmConfig& ecm = arms->get(i);
            if (!ecm.type.isECM()) { continue; }

            for (int j = 0; j < n; j++) {
                const ArmConfig& mtml = arms->get(j);
                if (!mtml.type.isMTM()) { continue; }

                // start at j+1 so we only add one of e.g. ECM-MTML-MTMR and ECM-MTMR-MTML
                for (int k = j + 1; k < n; k++) {
                    const ArmConfig& mtmr = arms->get(k);
                    if (!mtmr.type.isMTM()) { continue; }

                    TeleopConfig teleop = TeleopConfig("ECM Teleop", TeleopType::Value::ECM_TELEOP);
                    teleop.arm_names = { ecm.name, mtml.name, mtmr.name };

                    add_candidate(teleop);
                }
            }
        }
    }

    suggested_teleops_view->updateGeometry();
}

bool SuggestedTeleopsPage::isComplete() const {
    auto selections = suggested_teleops_view->selectedItems();
    for (bool selected : selections) {
        if (selected) {
            return true;
        }
    }

    return false;
}

TeleopParametersPage::TeleopParametersPage(TeleopConfig& config, const ListModelT<ArmConfig>& arms, QWidget *parent)
    : QWizardPage(parent), config(&config), arms(&arms) {
    setTitle("Teleop parameters");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QFormLayout* common_form = new QFormLayout();

    scale_selector = new QSlider(Qt::Orientation::Horizontal);
    scale_selector->setRange(5, 65);
    scale_selector->setToolTip("Reduction factor from MTM motion to PSM motion");
    scale_selector->setTickPosition(QSlider::TicksAbove);
    scale_selector->setTickInterval(10);
    scale_selector->setSingleStep(10);

    QWidget* scale_wrapper = new QWidget();
    QHBoxLayout* scale_layout = new QHBoxLayout(scale_wrapper);
    scale_layout->setAlignment(Qt::AlignCenter);
    QLabel* scale_display = new QLabel("0.30");
    scale_layout->addWidget(scale_display);
    scale_layout->addWidget(scale_selector);
    scale_layout->setMargin(0);
    scale_layout->setContentsMargins(0, 0, 0, 0);

    QObject::connect(scale_selector, &QSlider::valueChanged, this, [this, scale_display](int value) {
        this->config->scale = static_cast<double>(value) / 100.0;

        // format as 0.yz
        std::ostringstream scale_string;
        scale_string << std::fixed << std::setprecision(2) << this->config->scale;
        QString value_str = QString::fromStdString(scale_string.str());

        // update display label with new value
        scale_display->setText(value_str);
    });

    QLabel* description = new QLabel("MTM movements are reduced by a scale factor from 0.05 to 0.65 before they are sent to the teloperated arm.");
    description->setWordWrap(true);
    common_form->addRow(description);
    common_form->addRow("Scale factor:", scale_wrapper);
    layout->addLayout(common_form);
    layout->addSpacing(10);

    haptic_mtm_details = new QWidget();
    QFormLayout* haptic_mtm_form = new QFormLayout(haptic_mtm_details);
    has_gripper = new QCheckBox();
    haptic_mtm_form->addRow("Does your haptic device MTM have a gripper?", has_gripper);
    QObject::connect(has_gripper, &QCheckBox::toggled, this, [&config](bool checked) {
        config.has_MTM_gripper = checked;
    });

    has_actuated_wrist = new QCheckBox();
    haptic_mtm_form->addRow("Does your haptic device MTM have a motorized wrist mechanism?", has_actuated_wrist);
    QObject::connect(has_actuated_wrist, &QCheckBox::toggled, this, [&config](bool checked) {
        config.has_MTM_wrist_actuation = checked;
    });

    haptic_mtm_form->setMargin(0);
    layout->addWidget(haptic_mtm_details);

    psm_base_frame_details = new QWidget();
    QFormLayout* psm_base_frame_form = new QFormLayout(psm_base_frame_details);
    base_arms = new QComboBox();
    base_arms->addItem("None", 0);

    psm_base_frame_form->addRow("Which arm do you want to use as the reference frame?", base_arms);
    QObject::connect(base_arms, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (block_psm_base_frame_updates) { return; }

        if (index == 0) {
            this->config->PSM_base_frame = {};
        } else  {
            this->config->PSM_base_frame = ComponentInterfaceConfig();
            this->config->PSM_base_frame->component_name = "ECM";
            this->config->PSM_base_frame->interface_name = "Arm";
        }
    });

    psm_base_frame_form->setMargin(0);
    layout->addWidget(psm_base_frame_details);
}

void TeleopParametersPage::showEvent(QShowEvent *CMN_UNUSED(event)) {
    scale_selector->setValue(std::round(100.0 * config->scale));

    bool haptic_mtm = usesHapticMTM();
    haptic_mtm_details->setVisible(haptic_mtm);
    if (haptic_mtm) {
        has_gripper->setChecked(config->has_MTM_gripper);
        has_actuated_wrist->setChecked(config->has_MTM_wrist_actuation);
    } else {
        config->has_MTM_gripper = true;
        config->has_MTM_wrist_actuation = true;
    }

    psm_base_frame_details->setVisible(config->type.isPSM());
    if (config->type.isPSM()) {
        // temporarily block handling changes to base frame, otherwise the
        // the base_arms->addItem(...) calls will overwrite the config
        block_psm_base_frame_updates = true;
        base_arms->clear();
        base_arms->addItem("None", 0);

        int base_arm_idx = 1;
        for (int idx = 0; idx < arms->count(); idx++) {
            const auto& arm = arms->get(idx);
            if (arm.type.isPSM() || arm.type.isECM()) {
                base_arms->addItem(QString::fromStdString(arm.name), base_arm_idx++);
            }
        }

        if (!config->PSM_base_frame.has_value()) {
            base_arms->setCurrentIndex(0);
        } else {
            base_arms->setCurrentText(QString::fromStdString(config->PSM_base_frame->component_name));
        }

        block_psm_base_frame_updates = false;
    }
}

bool TeleopParametersPage::usesHapticMTM() const {
    /** Check if haptic device is being used as an MTM */
    if (!config->type.isPSM()) {
        return false;
    }

    for (const auto& name : config->arm_names) {
        auto arm = config->getSource(name);
        if (!arm) { continue; }

        if (arm->config_type == ArmConfigType::HAPTIC_MTM) {
            return true;
        }
    }

    return false;
}

}

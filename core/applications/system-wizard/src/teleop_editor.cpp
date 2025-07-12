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
#include <sstream>

namespace system_wizard {

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
    : QWizard(parent), teleops(&teleops), type(type), config("Teleop", type) {
    SuggestedTeleopsPage* suggested_teleops_page = new SuggestedTeleopsPage(arms, teleops, config);
    TeleopParametersPage* teleop_parameters_page = new TeleopParametersPage(config);

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
    } else {
        setStartId(PAGE_SUGGESTED_TELEOPS);
    }
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

TeleopParametersPage::TeleopParametersPage(TeleopConfig& config, QWidget *parent)
    : QWizardPage(parent), config(&config) {
    setTitle("Teleop parameters");

    QFormLayout* form = new QFormLayout(this);

    scale_selector = new QSlider(Qt::Orientation::Horizontal);
    scale_selector->setRange(0, 100);
    scale_selector->setToolTip("Reduction factor from MTM motion to PSM motion");
    scale_selector->setTickInterval(25);
    scale_selector->setTickPosition(QSlider::TicksAbove);
    scale_selector->setSingleStep(25);

    QObject::connect(scale_selector, &QSlider::valueChanged, this, [this](int value) {
        this->config->scale = static_cast<double>(value) / 100.0;

        std::ostringstream scale_string;
        scale_string << std::setprecision(2) << this->config->scale;
        QString value_str = QString::fromStdString(scale_string.str());

        int x_pos = QStyle::sliderPositionFromValue(0, 100, value, this->scale_selector->width());
        QPoint slider_handle = QPoint(x_pos, 0);
        QToolTip::showText(this->scale_selector->mapToGlobal(slider_handle), value_str);
    });

    QLabel* description = new QLabel("MTM movements are scaled by a scale factor from 0.0 to 1.0 before they are sent to the teloperated arm.");
    description->setWordWrap(true);
    form->addRow(description);
    form->addRow("Scale factor:", scale_selector);
}

void TeleopParametersPage::initializePage() {
    scale_selector->setValue(std::round(100.0 * config->scale));
}

}

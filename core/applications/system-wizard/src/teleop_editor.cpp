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

namespace system_wizard {

PSMTeleopOptionView::PSMTeleopOptionView(ListModelT<TeleopConfig>& model, ListView& list_view, int index, QWidget* parent)
    : ItemView(list_view, index, parent), model(&model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(index);

    layout->addWidget(display);
}

void PSMTeleopOptionView::updateData(int index) {
    const auto& teleop = model->get(index);
    std::string arms = teleop.arm_names[0] + ", " + teleop.arm_names[1];
    QString text = QString::fromStdString(teleop.type.name() + ": " + arms);
    display->setText(text);
}

ECMTeleopOptionView::ECMTeleopOptionView(ListModelT<TeleopConfig>& model, ListView& list_view, int index, QWidget* parent)
    : ItemView(list_view, index, parent), model(&model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(index);

    layout->addWidget(display);
}

void ECMTeleopOptionView::updateData(int index) {
    const auto& teleop = model->get(index);
    std::string arms = teleop.arm_names[0] + ", " + teleop.arm_names[1] + ", " + teleop.arm_names[2];
    QString text = QString::fromStdString(teleop.type.name() + ": " + arms);
    display->setText(text);
}

TeleopEditor::TeleopEditor(ConsoleConfig& console, const ListModelT<ArmConfig>& arms, QWidget* parent)
    : QWizard(parent), console(&console), config("Teleop", TeleopType::Value::PSM_TELEOP) {
    suggested_teleops_page = new SuggestedTeleopsPage(arms, config);
    
    setPage(PAGE_SUGGESTED_TELEOPS, suggested_teleops_page);

    setWizardStyle(QWizard::ModernStyle);
    setOption(QWizard::NoBackButtonOnStartPage);
    setWindowTitle("Teleop Config Editor");

    setStartId(PAGE_SUGGESTED_TELEOPS);

    setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);

    QObject::connect(this, &QDialog::accepted, this, &TeleopEditor::save);
}

void TeleopEditor::setId(bool psm, int index) {
    this->psm = psm;
    this->index = index;

    suggested_teleops_page->setMode(psm);

    if (index >= 0) {
        if (psm) {
            config = console->psm_teleops->get(index);
        } else {
            config = console->ecm_teleops->get(index);
        }

        // if (config.type == TeleopType::Value::PSM_TELEOP) {
        //     setStartId(PAGE_PSM_TELEOP);
        // } else if (config.type == TeleopType::Value::ECM_TELEOP) {
        //     setStartId(PAGE_ECM_TELEOP);
        // } else {
        //     Q_ASSERT(false);
        //     return;
        // }
    } else {
        setStartId(PAGE_SUGGESTED_TELEOPS);
    }
}

void TeleopEditor::save() {
    if (index < 0) {
        if (psm) {
            console->psm_teleops->appendItem(config);
        } else {
            console->ecm_teleops->appendItem(config);
        }
    } else {
        if (psm) {
            console->psm_teleops->ref(index) = config;
            console->psm_teleops->updateItem(index);
        } else {
            console->ecm_teleops->ref(index) = config;
            console->ecm_teleops->updateItem(index);
        }
    }
}

SuggestedTeleopsPage::SuggestedTeleopsPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent)
    : QWizardPage(parent), config(&config), arms(&arms) {
    setTitle("Suggested teleops");
    setSubTitle("Suggested teleops are based on available arms");

    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel* source_label = new QLabel("Choose from suggested teleops:");
    source_label->setWordWrap(true);
    layout->addWidget(source_label);

    auto suggested_teleop_factory = [this](int index, ListView& view) -> std::unique_ptr<ItemView> {
        if (this->psm_mode) {
            return std::make_unique<PSMTeleopOptionView>(suggested_teleops, view, index);
        } else {
            return std::make_unique<ECMTeleopOptionView>(suggested_teleops, view, index);
        }
    };
    suggested_teleops_view = new ListView(suggested_teleops, suggested_teleop_factory, SelectionMode::SINGLE);
    suggested_teleops_view->setEmptyMessage("No suggestions available");
    QObject::connect(suggested_teleops_view, &ListView::choose, this, [this, &model=suggested_teleops](int index){
        *this->config = model.get(index);
        this->wizard()->accept();
    });
    QObject::connect(suggested_teleops_view, &ListView::selected, this, [this, &model=suggested_teleops](int index, bool selected){
        if (selected) {
            *this->config = model.get(index);
            next_page_id = -1;
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
    next_page_id = -1;
    suggested_teleops.clear();

    int n = arms->count();

    if (psm_mode) {
        // PSM teleops
        for (int i = 0; i < n; i++) {
            const ArmConfig& mtm = arms->get(i);
            if (!mtm.type.isMTM()) { continue; }

            for (int j = 0; j < n; j++) {
                const ArmConfig& psm = arms->get(j);
                if (!psm.type.isPSM()) { continue; }
                
                TeleopConfig teleop = TeleopConfig("Test", TeleopType::Value::PSM_TELEOP);
                teleop.arm_names = { mtm.name, psm.name };
                suggested_teleops.appendItem(teleop);
            }
        }
    } else {
        // ECM teleops
        for (int i = 0; i < n; i++) {
            const ArmConfig& ecm = arms->get(i);
            if (!ecm.type.isECM()) { continue; }

            for (int j = 0; j < n; j++) {
                const ArmConfig& mtml = arms->get(j);
                if (!mtml.type.isMTM()) { continue; }

                for (int k = j + 1; k < n; k++) {
                    const ArmConfig& mtmr = arms->get(k);
                    if (!mtmr.type.isMTM()) { continue; }

                    TeleopConfig teleop = TeleopConfig("Test", TeleopType::Value::ECM_TELEOP);
                    teleop.arm_names = { ecm.name, mtml.name, mtmr.name };
                    suggested_teleops.appendItem(teleop);
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

// PSMTeleopPage::PSMTeleopPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent)
//     : QWizardPage(parent), arms(&arms), config(&config) {
//     setTitle("PSM Teleop");

//     QFormLayout* form = new QFormLayout(this);

//     mtms = new QComboBox();
//     mtms->setPlaceholderText("select MTM");

//     psms = new QComboBox();
//     psms->setPlaceholderText("select PSM");

//     form->addRow("MTM:", mtms);
//     form->addRow("PSM:", psms);
// }

// void PSMTeleopPage::initializePage() {
//     mtms->clear();
//     for (int i = 0; i < arms->count(); i++) {
//         const ArmConfig& mtm = arms->get(i);
//         if (!mtm.type.isMTM()) { continue; }

//         mtms->addItem(QString::fromStdString(mtm.name), i);
//     }

//     psms->clear();
//     for (int i = 0; i < arms->count(); i++) {
//         const ArmConfig& psm = arms->get(i);
//         if (!psm.type.isPSM()) { continue; }

//         psms->addItem(QString::fromStdString(psm.name), i);
//     }
// }

// ECMTeleopPage::ECMTeleopPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent)
//     : QWizardPage(parent), arms(&arms), config(&config) {
//     setTitle("ECM Teleop");

//     QFormLayout* form = new QFormLayout(this);

//     ecms = new QComboBox();
//     ecms->setPlaceholderText("select ECM");

//     left_mtms = new QComboBox();
//     right_mtms = new QComboBox();
//     left_mtms->setPlaceholderText("select left MTM");
//     right_mtms->setPlaceholderText("select right MTM");

//     form->addRow("ECM:", ecms);
//     form->addRow("Left MTM:", left_mtms);
//     form->addRow("Right MTM:", right_mtms);
// }

// void ECMTeleopPage::initializePage() {
//     ecms->clear();
//     for (int i = 0; i < arms->count(); i++) {
//         const ArmConfig& ecm = arms->get(i);
//         if (!ecm.type.isECM()) { continue; }

//         ecms->addItem(QString::fromStdString(ecm.name), i);
//     }

//     left_mtms->clear();
//     right_mtms->clear();
//     for (int i = 0; i < arms->count(); i++) {
//         const ArmConfig& mtm = arms->get(i);
//         if (!mtm.type.isMTM()) { continue; }

//         left_mtms->addItem(QString::fromStdString(mtm.name), i);
//         right_mtms->addItem(QString::fromStdString(mtm.name), i);
//     }
// }

}

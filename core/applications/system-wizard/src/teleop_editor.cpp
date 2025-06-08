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

TeleopOptionView::TeleopOptionView(const ListModelT<ArmConfig>& arms, ListModelT<TeleopConfig>& model, ListView& list_view, int id, QWidget* parent)
    : ItemView(list_view, id, parent), arms(&arms), model(&model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(id);

    layout->addWidget(display);
}

void TeleopOptionView::updateData(int id) {
    const TeleopConfig& teleop = model->get(id);
    std::string arm_names = std::accumulate(teleop.arms.begin(), teleop.arms.end(), std::string(""), [this](std::string acc, int arm_id) -> std::string {
        std::string arm_name = this->arms->get(arm_id).name;
        return acc.empty() ? arm_name : acc + ", " + arm_name;
    });

    QString text = QString::fromStdString(teleop.type.name() + ": " + arm_names);
    display->setText(text);
}

TeleopOptionViewFactory::TeleopOptionViewFactory(const ListModelT<ArmConfig>& arms, ListModelT<TeleopConfig>& model)
    : arms(&arms), model(&model) {}

TeleopOptionView* TeleopOptionViewFactory::create(int id, ListView& list_view) {
    return new TeleopOptionView(*arms, *model, list_view, id);
}

TeleopEditor::TeleopEditor(SystemConfigModel& model, QWidget* parent)
    : QWizard(parent), model(&model), config("Teleop", TeleopType::Value::PSM_TELEOP) {
    setPage(PAGE_SUGGESTED_TELEOPS, new SuggestedTeleopsPage(model.arm_configs, config));
    setPage(PAGE_PSM_TELEOP, new PSMTeleopPage(model.arm_configs, config));
    setPage(PAGE_ECM_TELEOP, new ECMTeleopPage(model.arm_configs, config));

    setWizardStyle(QWizard::ModernStyle);
    setOption(QWizard::NoBackButtonOnStartPage);
    setWindowTitle("Teleop Config Editor");

    setStartId(PAGE_SUGGESTED_TELEOPS);

    setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);

    QObject::connect(this, &QDialog::accepted, this, &TeleopEditor::done);
}

void TeleopEditor::setId(int id) {
    this->id = id;

    if (id >= 0) {
        config = model->teleop_configs.get(id);

        if (config.type == TeleopType::Value::PSM_TELEOP) {
            setStartId(PAGE_PSM_TELEOP);
        } else if (config.type == TeleopType::Value::ECM_TELEOP) {
            setStartId(PAGE_ECM_TELEOP);
        } else {
            Q_ASSERT(false);
            return;
        }
    } else {
        setStartId(PAGE_SUGGESTED_TELEOPS);
    }
}

void TeleopEditor::done() {
    if (id < 0) {
        model->teleop_configs.addItem(config);
    } else {
        model->teleop_configs.updateItem(id, config);
    }
}

SuggestedTeleopsPage::SuggestedTeleopsPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent)
    : QWizardPage(parent), config(&config), arms(&arms), suggested_teleop_factory(arms, suggested_teleops) {
    setTitle("Suggested teleops");
    setSubTitle("Suggested teleops are based on available arms");

    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel* source_label = new QLabel("Choose from suggested teleops:");
    source_label->setWordWrap(true);
    layout->addWidget(source_label);

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

    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);

    QLabel* custom_label = new QLabel("or configure a different one:");
    custom_label->setWordWrap(true);
    layout->addWidget(custom_label);
    QHBoxLayout* custom_teleop_layout = new QHBoxLayout();
    QPushButton* psm_teleop_button = new QPushButton("Configure PSM teleop");
    QPushButton* ecm_teleop_button = new QPushButton("Configure ECM teleop");
    custom_teleop_layout->addStretch();
    custom_teleop_layout->addWidget(psm_teleop_button);
    custom_teleop_layout->addStretch();
    custom_teleop_layout->addWidget(ecm_teleop_button);
    custom_teleop_layout->addStretch();
    layout->addLayout(custom_teleop_layout);

    QObject::connect(psm_teleop_button, &QPushButton::clicked, this, [this]() {
        if (wizard() != nullptr) {
            next_page_id = TeleopEditor::PAGE_PSM_TELEOP;
            wizard()->next();
            next_page_id = -1;
        }
    });
    QObject::connect(ecm_teleop_button, &QPushButton::clicked, this, [this]() {
        if (wizard() != nullptr) {
            next_page_id = TeleopEditor::PAGE_ECM_TELEOP;
            wizard()->next();
            next_page_id = -1;
        }
    });

    layout->addStretch();

    // prevent arm list from being stretched out after items are removed
    suggested_teleops_view->layout()->setSizeConstraint(QLayout::SetMinimumSize);
}

void SuggestedTeleopsPage::initializePage() {
    suggested_teleops_view->clearSelections();
    next_page_id = -1;
    suggested_teleops.clear();

    // PSM teleops
    int n = arms->count();
    for (int i = 0; i < n; i++) {
        const ArmConfig& mtm = arms->get(i);
        if (!mtm.type.isMTM()) { continue; }

        for (int j = 0; j < n; j++) {
            const ArmConfig& psm = arms->get(j);
            if (!psm.type.isPSM()) { continue; }
            
            TeleopConfig teleop = TeleopConfig("Test", TeleopType::Value::PSM_TELEOP);
            teleop.arms = { i, j };
            suggested_teleops.addItem(teleop);
        }
    }

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
                teleop.arms = { i, j, k };
                suggested_teleops.addItem(teleop);
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

PSMTeleopPage::PSMTeleopPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent)
    : QWizardPage(parent), arms(&arms), config(&config) {
    setTitle("PSM Teleop");

    QFormLayout* form = new QFormLayout(this);

    mtms = new QComboBox();
    mtms->setPlaceholderText("select MTM");

    psms = new QComboBox();
    psms->setPlaceholderText("select PSM");

    form->addRow("MTM:", mtms);
    form->addRow("PSM:", psms);
}

void PSMTeleopPage::initializePage() {
    mtms->clear();
    for (int i = 0; i < arms->count(); i++) {
        const ArmConfig& mtm = arms->get(i);
        if (!mtm.type.isMTM()) { continue; }

        mtms->addItem(QString::fromStdString(mtm.name), i);
    }

    psms->clear();
    for (int i = 0; i < arms->count(); i++) {
        const ArmConfig& psm = arms->get(i);
        if (!psm.type.isPSM()) { continue; }

        psms->addItem(QString::fromStdString(psm.name), i);
    }
}

ECMTeleopPage::ECMTeleopPage(const ListModelT<ArmConfig>& arms, TeleopConfig& config, QWidget *parent)
    : QWizardPage(parent), arms(&arms), config(&config) {
    setTitle("ECM Teleop");

    QFormLayout* form = new QFormLayout(this);

    ecms = new QComboBox();
    ecms->setPlaceholderText("select ECM");

    left_mtms = new QComboBox();
    right_mtms = new QComboBox();
    left_mtms->setPlaceholderText("select left MTM");
    right_mtms->setPlaceholderText("select right MTM");

    form->addRow("ECM:", ecms);
    form->addRow("Left MTM:", left_mtms);
    form->addRow("Right MTM:", right_mtms);
}

void ECMTeleopPage::initializePage() {
    ecms->clear();
    for (int i = 0; i < arms->count(); i++) {
        const ArmConfig& ecm = arms->get(i);
        if (!ecm.type.isECM()) { continue; }

        ecms->addItem(QString::fromStdString(ecm.name), i);
    }

    left_mtms->clear();
    right_mtms->clear();
    for (int i = 0; i < arms->count(); i++) {
        const ArmConfig& mtm = arms->get(i);
        if (!mtm.type.isMTM()) { continue; }

        left_mtms->addItem(QString::fromStdString(mtm.name), i);
        right_mtms->addItem(QString::fromStdString(mtm.name), i);
    }
}

}

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

namespace system_wizard {

class ArmSourceView : public ItemView {
public:
    ArmSourceView(ListModelT<ConfigSources::Arm>* model, ListView& list_view, int id, QWidget* parent = nullptr)
    : ItemView(list_view, id, parent), model(model) {
        QHBoxLayout* layout = new QHBoxLayout(this);

        display = new QLabel();
        updateData(id);

        layout->addWidget(display);
    }

public slots:
    void updateData(int id) override {
        ConfigSources::Arm arm = model->get(id);
        QString description = QString::fromStdString(arm.type + "-" + arm.serial_number);
        display->setText(description);
    }

private:
    ListModelT<ConfigSources::Arm>* model;
    QLabel* display;
};

class ArmSourceViewFactory : public ItemViewFactory {
public:
    ArmSourceViewFactory(ListModelT<ConfigSources::Arm>* model) : model(model) {}

    ArmSourceView* create(int id, ListView& list_view) {
        return new ArmSourceView(model, list_view, id);
    }

private:
    ListModelT<ConfigSources::Arm>* model;
};

QuickArmPage::QuickArmPage(ConfigSources* config_sources, QWidget *parent)
    : QWizardPage(parent), config_sources(config_sources) {
    setTitle("Quick arm");
    setSubTitle("Choose from default arms");

    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel* lorem_ipsum = new QLabel("Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.");
    lorem_ipsum->setWordWrap(true);
    layout->addWidget(lorem_ipsum);

    setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);

    factory = std::make_unique<ArmSourceViewFactory>(&config_sources->getModel());
    arm_list_view = new ListView(&config_sources->getModel(), factory.get(), SelectionMode::SINGLE);
    layout->addWidget(arm_list_view);
    layout->addStretch();

    // prevent arm list from being stretched out after items are removed
    arm_list_view->layout()->setSizeConstraint(QLayout::SetMinimumSize);
}

void QuickArmPage::initializePage() {
    // make sure dialog size is updated if arm source list has changed while hidden
    arm_list_view->updateGeometry();
    arm_list_view->clearSelections();
}

BasicArmPage::BasicArmPage(QWidget *parent) : QWizardPage(parent) {
    setTitle("Basic arm info");

    QFormLayout* form = new QFormLayout(this);

    name_input = new QLineEdit();
    registerField("basic.name*", name_input);

    type_selector = new QComboBox();
    type_selector->setModel(&type_model);
    registerField("basic.type*", type_selector);

    form->addRow("Name:", name_input);
    form->addRow("Type:", type_selector);
}

ArmEditor::ArmEditor(SystemConfigModel* model, ConfigSources* config_sources, QWidget* parent)
    : QWizard(parent), model(model), config_sources(config_sources) {
    setPage(PAGE_QUICK_ARM, new QuickArmPage(config_sources));
    setPage(PAGE_BASIC, new BasicArmPage());

    setWizardStyle(QWizard::ModernStyle);
    setOption(QWizard::NoBackButtonOnStartPage);
    setWindowTitle("Arm Config Editor");

    setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);

    setStartId(PAGE_QUICK_ARM);

    QObject::connect(this, &QDialog::accepted, this, &ArmEditor::done);
}

void ArmEditor::done() {
    ArmConfig config = ArmConfig("Test", ArmType::Value::PSM_SOCKET);
    model->arm_configs.addItem(config);
}

}

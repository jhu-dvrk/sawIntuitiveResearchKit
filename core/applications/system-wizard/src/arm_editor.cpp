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

ArmSourceView::ArmSourceView(ListModelT<ConfigSources::Arm>& model, ListView& list_view, int id, QWidget* parent)
: ItemView(list_view, id, parent), model(&model) {
    QHBoxLayout* layout = new QHBoxLayout(this);

    display = new QLabel();
    updateData(id);

    layout->addWidget(display);
}

void ArmSourceView::updateData(int id) {
    ConfigSources::Arm arm = model->get(id);
    QString description = QString::fromStdString(arm.name + "-" + arm.serial_number);
    display->setText(description);
}

ArmSourceViewFactory::ArmSourceViewFactory(ListModelT<ConfigSources::Arm>& model) : model(&model) {}

ArmSourceView* ArmSourceViewFactory::create(int id, ListView& list_view) {
    return new ArmSourceView(*model, list_view, id);
}

QuickArmPage::QuickArmPage(ArmEditor& editor, ConfigSources& config_sources, QWidget *parent)
    : QWizardPage(parent), editor(&editor), arm_list_factory(config_sources.getModel()) {
    setTitle("Quick arm");
    setSubTitle("Choose from default arms");
    setFinalPage(true);

    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel* source_label = new QLabel("Choose from available arms:");
    source_label->setWordWrap(true);
    layout->addWidget(source_label);

    arm_list_view = new ListView(config_sources.getModel(), arm_list_factory, SelectionMode::SINGLE);
    arm_list_view->setEmptyMessage("No arms available - open a config folder");
    QObject::connect(arm_list_view, &ListView::choose, this, [&editor, &model=config_sources.getModel()](int index){
        ConfigSources::Arm source = model.get(index);
        editor.selectArmSource(source);
    });
    QObject::connect(arm_list_view, &ListView::selected, this, &QWizardPage::completeChanged);

    layout->addWidget(arm_list_view);

    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);

    QLabel* custom_label = new QLabel("or add another type of arm, e.g. Falcon/Omni or arm from simulation");
    custom_label->setWordWrap(true);
    layout->addWidget(custom_label);
    QHBoxLayout* custom_arm_layout = new QHBoxLayout();
    QPushButton* custom_arm_button = new QPushButton("Custom arm");
    custom_arm_layout->addStretch();
    custom_arm_layout->addWidget(custom_arm_button);
    layout->addLayout(custom_arm_layout);

    layout->addStretch();

    // prevent arm list from being stretched out after items are removed
    arm_list_view->layout()->setSizeConstraint(QLayout::SetMinimumSize);
}

void QuickArmPage::initializePage() {
    arm_list_view->clearSelections();

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

ArmEditor::ArmEditor(SystemConfigModel& model, ConfigSources& config_sources, QWidget* parent)
    : QWizard(parent), model(&model) {
    setPage(PAGE_QUICK_ARM, new QuickArmPage(*this, config_sources));
    setPage(PAGE_BASIC, new BasicArmPage());

    QList<QWizard::WizardButton> button_layout;
    button_layout << QWizard::Stretch << QWizard::FinishButton << QWizard::CancelButton;
    setButtonLayout(button_layout);

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

void ArmEditor::selectArmSource(ConfigSources::Arm arm_source) {
    ArmConfig config = ArmConfig(arm_source.name, arm_source.type);
    config.serial_number = arm_source.serial_number;
    config.interface_name = "Arm";

    model->arm_configs.addItem(config);
    close();
}

}

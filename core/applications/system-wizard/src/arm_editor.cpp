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

#include <cisstCommon/cmnPortability.h>

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

ArmSourceViewFactory::ArmSourceViewFactory(ListModelT<ConfigSources::Arm>& model) : model(&model) {}

ArmSourceView* ArmSourceViewFactory::create(int id, ListView& list_view) {
    return new ArmSourceView(*model, list_view, id);
}

QuickArmPage::QuickArmPage(ArmConfig& config, SystemConfigModel& model, ConfigSources& config_sources, QWidget *parent)
    : QWizardPage(parent),
      config(&config),
      model(&model),
      config_sources(&config_sources),
      available_arms(std::make_unique<VectorList<ConfigSources::Arm>>()),
      arm_list_factory(*available_arms) {
    setTitle("Quick arm");
    setSubTitle("Choose from default arms");

    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel* source_label = new QLabel("Choose from available arms:");
    source_label->setWordWrap(true);
    layout->addWidget(source_label);

    arm_list_view = new ListView(*available_arms, arm_list_factory, SelectionMode::SINGLE);
    arm_list_view->setEmptyMessage("No arms available - open a config folder");
    QObject::connect(arm_list_view, &ListView::choose, this, [this](int index){
        ConfigSources::Arm source = available_arms->get(index);
        *this->config = ArmConfig(source.name, source.type, ArmConfigType::NATIVE);
        this->config->serial_number = source.serial_number;
        this->config->interface_name = "Arm";

        next_page_id = -1;
        this->wizard()->accept();
    });
    QObject::connect(arm_list_view, &ListView::selected, this, [this](int index, bool selected){
        if (selected) {
            next_page_id = -1;

            ConfigSources::Arm source = available_arms->get(index);
            *this->config = ArmConfig(source.name, source.type, ArmConfigType::NATIVE);
            this->config->serial_number = source.serial_number;
            this->config->interface_name = "Arm";
        }
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

    available_arms->update(available);

    arm_list_view->clearSelections();

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
    QObject::connect(haptic_input, &QPushButton::clicked, this, [this]() {
        this->config->config_type = ArmConfigType::HAPTIC_MTM;
        this->next_page_id = ArmEditor::PAGE_HAPTIC_MTM;
        this->wizard()->next();
    });

    haptic_input->setFlat(true);
    haptic_input->setAutoFillBackground(true);
    layout->addWidget(haptic_input);
    layout->addSpacing(10);
    layout->addWidget(new QLabel("If you want to add a remote or simulated PSM/MTM arm available via ROS:"));
    QPushButton* via_ros = new QPushButton("Client arm for remote/simulated ROS arm");
    QObject::connect(via_ros, &QPushButton::clicked, this, [this]() {
        this->config->config_type = ArmConfigType::ROS_ARM;
        this->next_page_id = ArmEditor::PAGE_ROS_ARM;
        this->wizard()->next();
    });

    via_ros->setFlat(true);
    via_ros->setAutoFillBackground(true);
    layout->addWidget(via_ros);
}

HapticMTMPage::HapticMTMPage(ArmConfig& config, QWidget *parent) : QWizardPage(parent), config(&config) {
    setTitle("Configure haptic device as MTM");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QFormLayout* device_type_form = new QFormLayout();

    // Haptic device selector determines which config options we display below,
    // as well as what shared library component we need to load
    haptic_device_selector = new QComboBox();
    haptic_device_selector->setPlaceholderText("select haptic device type");
    haptic_device_selector->addItem("Novint Falcon",               0);
    haptic_device_selector->addItem("ForceDimension omega/delta",  1);
    haptic_device_selector->addItem("ForceDimension sigma/lambda", 2);
    haptic_device_selector->addItem("Phantom Omni/Geomagic Touch", 3);
    haptic_device_selector->addItem("Other",                       4);
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
    left_right_selector->setPlaceholderText("select left/right input");
    left_right_selector->addItem("MTML (left input arm)",  0);
    left_right_selector->addItem("MTMR (right input arm)", 1);

    force_dimension_layout->addWidget(left_right_label);
    force_dimension_layout->addWidget(left_right_selector);

    QHBoxLayout* config_file_selector = new QHBoxLayout();
    QFileDialog* file_dialog = new QFileDialog(this);
    file_dialog->setFileMode(QFileDialog::ExistingFile);
    file_dialog->setViewMode(QFileDialog::List);
    file_dialog->setOptions(QFileDialog::DontResolveSymlinks);
    QLineEdit* config_file_name = new QLineEdit();
    QPushButton* config_file_browse_button = new QPushButton("Browse");
    QObject::connect(config_file_browse_button, &QPushButton::clicked, file_dialog, &QDialog::open);
    QObject::connect(file_dialog, &QFileDialog::fileSelected, this, [config_file_name](const QString& file_name) {
        if (file_name.isEmpty()) { return; }
        config_file_name->setText(file_name);
    });
    config_file_name->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    config_file_selector->addWidget(config_file_name);
    config_file_selector->addWidget(config_file_browse_button);
    force_dimension_layout->addLayout(config_file_selector);

    QObject::connect(left_right_selector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, config_file_name](int index) {
        if (index == 0) {
            this->config->name = "MTML";
            this->config->interface_name = "MTML";
            config_file_name->setText("sawForceDimensionSDK-MTML.json");
        } else if (index == 1) {
            this->config->name = "MTMR";
            this->config->interface_name = "MTMR";
            config_file_name->setText("sawForceDimensionSDK-MTMR.json");
        }
    });

    details->addWidget(force_dimension);

    layout->addWidget(details);
    layout->addStretch();

    details->setCurrentIndex(0);
    QObject::connect(haptic_device_selector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        switch (index) {
        case 0:
        case 1:
        case 2:
            details->setCurrentIndex(1);
            this->config->haptic_device = index;
            break;
        default:
            details->setCurrentIndex(0);
            break;
        }
    });

    registerField("haptic.device*", haptic_device_selector);
}

void HapticMTMPage::initializePage() {
    haptic_device_selector->setCurrentIndex(-1);
    details->setCurrentIndex(0);
    left_right_selector->setCurrentIndex(-1);
}

void HapticMTMPage::showEvent(QShowEvent *CMN_UNUSED(event)) {
    if (config->config_type != ArmConfigType::HAPTIC_MTM) {
        *config = ArmConfig("MTM", ArmType::Value::MTM_GENERIC, ArmConfigType::HAPTIC_MTM);
    } else {
        config->type = ArmType::Value::MTM_GENERIC;
        if (config->haptic_device) {
            haptic_device_selector->setCurrentIndex(config->haptic_device.value());
        }

        if (config->interface_name == "MTML") {
            left_right_selector->setCurrentIndex(0);
        } else if (config->interface_name == "MTMR") {
            left_right_selector->setCurrentIndex(1);
        }
    }
}

ROSArmPage::ROSArmPage(ArmConfig& config, QWidget *parent) : QWizardPage(parent), config(&config) {
    setTitle("Remote arm via ROS");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* description1 = new QLabel("Client arm for a remote dVRK arm available via ROS, either an actual arm or a simulated provided by e.g. the AMBF simulator.");
    description1->setWordWrap(true);
    layout->addWidget(description1);
    QLabel* description2 = new QLabel("Client arm name (e.g. \"PSM1\") must match ROS namespace used for remote arm");
    description2->setWordWrap(true);
    layout->addWidget(description2);

    QFormLayout* form = new QFormLayout();

    arm_name = new QLineEdit();
    form->addRow("Arm name:", arm_name);
    QObject::connect(arm_name, &QLineEdit::textChanged, this, [this](const QString& text){
        this->config->name = text.toStdString();
    });

    arm_type = new QComboBox();
    arm_type->setPlaceholderText("select arm type");
    arm_type->addItem("PSM", 0);
    arm_type->addItem("MTM", 1);
    QObject::connect(arm_type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (index == 0) {
            this->config->type = ArmType::Value::PSM_GENERIC;
            this->config->config_type = ArmConfigType::ROS_ARM;
        } else if (index == 1) {
            this->config->type = ArmType::Value::MTM_GENERIC;
            this->config->config_type = ArmConfigType::ROS_ARM;
        }
    });

    form->addRow("Arm type:", arm_type);

    layout->addLayout(form);
}

void ROSArmPage::initializePage() { }

void ROSArmPage::showEvent(QShowEvent *CMN_UNUSED(event)) {
    if (config->config_type != ArmConfigType::ROS_ARM) {
        *config = ArmConfig("Arm", ArmType::Value::PSM_GENERIC, ArmConfigType::ROS_ARM);
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
        arm_name->setText(QString::fromStdString(config->name));
    }
}

ArmEditor::ArmEditor(SystemConfigModel& model, ConfigSources& config_sources, QWidget* parent)
    : QWizard(parent), model(&model), config("Arm", ArmType::Value::PSM, ArmConfigType::NATIVE) {
    setPage(PAGE_QUICK_ARM, new QuickArmPage(config, model, config_sources));
    setPage(PAGE_ARM_TYPE, new ArmTypePage(config));
    setPage(PAGE_HAPTIC_MTM, new HapticMTMPage(config));
    setPage(PAGE_ROS_ARM, new ROSArmPage(config));

    setWizardStyle(QWizard::ModernStyle);
    setOption(QWizard::NoBackButtonOnStartPage);
    setWindowTitle("Arm Config Editor");

    setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);

    setStartId(PAGE_QUICK_ARM);

    QObject::connect(this, &QDialog::accepted, this, &ArmEditor::done);
}

void ArmEditor::done() {
    if (id < 0) {
        model->arm_configs->addItem(config);
    } else {
        model->arm_configs->updateItem(id, config);
    }
}

void ArmEditor::setId(int id) {
    this->id = id;

    if (id >= 0) {
        config = model->arm_configs->get(id);

        switch (config.config_type) {
        case ArmConfigType::NATIVE:
            setStartId(PAGE_QUICK_ARM);
            break;
        case ArmConfigType::HAPTIC_MTM:
            setStartId(PAGE_HAPTIC_MTM);
            break;
        case ArmConfigType::ROS_ARM:
            setStartId(PAGE_ROS_ARM);
            break;
        default:
            Q_ASSERT(false);
            break;
        }
    } else {
        setStartId(PAGE_QUICK_ARM);
    }
}

}

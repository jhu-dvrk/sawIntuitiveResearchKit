/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-17

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "io_editor.hpp"

namespace system_wizard {

IOEditPage::IOEditPage(IOConfig& config, const SystemConfigModel& model, QWidget* parent)
    : QWizardPage(parent), config(&config), model(&model) {
    setTitle("I/O Config Editor");

    QFormLayout* form = new QFormLayout(this);

    name_input = new QLineEdit();
    QRegularExpression name_regex("[a-zA-Z]\\w*"); // must start with letter, followed by 0+ letters/digits
    QValidator *name_validator = new QRegularExpressionValidator(name_regex, this);
    name_input->setValidator(name_validator);
    name_invalid_msg = new QLabel();
    name_invalid_msg->setStyleSheet("color: red;");
    name_invalid_msg->hide();

    QObject::connect(name_input, &QLineEdit::textChanged, this, [this](){
        std::string new_name = this->name_input->text().toStdString();
        bool already_used = nameAlreadyUsed();
        if (already_used) {
            name_invalid_msg->setText("Name is already used by another io!");
            name_invalid_msg->show();
        } else {
            name_invalid_msg->hide();
            this->config->name = new_name;
        }
        emit completeChanged();
    });

    port_selector = new QComboBox();
    port_selector->setModel(&port_model);
    QObject::connect(port_selector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        this->config->port = static_cast<IOPort::Value>(index);
    });

    protocol_selector = new QComboBox();
    protocol_selector->setModel(&protocol_model);
    QObject::connect(protocol_selector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        this->config->protocol = static_cast<IOProtocol::Value>(index);
    });

    frequency_input = new QSpinBox();
    frequency_input->setRange(1, 10000);
    frequency_input->setSingleStep(100);
    frequency_input->setValue(1500);
    QObject::connect(frequency_input, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int hz) {
        this->config->period_ms = 1.0 / hz;
    });

    watchdog_timeout_input = new QDoubleSpinBox();
    watchdog_timeout_input->setRange(0.0, 5000.0);
    watchdog_timeout_input->setSingleStep(1.0);
    watchdog_timeout_input->setValue(10.0);
    QObject::connect(watchdog_timeout_input, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double timeout) {
        this->config->watchdog_timeout_ms = timeout;
    });

    QHBoxLayout* foot_pedals_layout = new QHBoxLayout();
    QFileDialog* file_dialog = new QFileDialog(this);
    file_dialog->setFileMode(QFileDialog::ExistingFile);
    file_dialog->setViewMode(QFileDialog::List);
    file_dialog->setOptions(QFileDialog::DontResolveSymlinks);
    foot_pedals_file = new QLineEdit();
    QRegularExpression file_regex("\\S*"); // zero or more non-whitespace
    QValidator *file_validator = new QRegularExpressionValidator(file_regex, this);
    foot_pedals_file->setValidator(file_validator);

    QPushButton* foot_pedals_browse_button = new QPushButton("Browse");
    QObject::connect(foot_pedals_browse_button, &QPushButton::clicked, file_dialog, &QDialog::open);
    QObject::connect(file_dialog, &QFileDialog::fileSelected, this, [this](const QString& file_name) {
        if (file_name.isEmpty()) { return; }
        this->foot_pedals_file->setText(file_name);
    });
    QObject::connect(foot_pedals_file, &QLineEdit::textChanged, this, [this](const QString& file_name) {
        if (file_name.isEmpty()) {
            this->config->foot_pedals = std::optional<std::filesystem::path>();
        } else {
            this->config->foot_pedals = file_name.toStdString();
        }
    });
    foot_pedals_file->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    foot_pedals_layout->addWidget(foot_pedals_file);
    foot_pedals_layout->addWidget(foot_pedals_browse_button);

    QLabel* name_label = new QLabel("Name:");
    name_label->setToolTip("I/O name - typically just \"io\"");
    form->addRow(name_label, name_input);
    form->addRow(name_invalid_msg);

    QLabel* port_label = new QLabel("Port:");
    port_label->setToolTip("Port information for communicating with controllers");
    form->addRow(port_label, port_selector);

    QLabel* protocol_label = new QLabel("Protocol:");
    protocol_label->setToolTip("Communication protocol");
    form->addRow(protocol_label, protocol_selector);

    QLabel* foot_pedals_label = new QLabel("Foot pedals:");
    foot_pedals_label->setToolTip("(Optional) config file for foot pedal inputs");
    form->addRow(foot_pedals_label, foot_pedals_layout);

    QLabel* frequency_label = new QLabel("Frequency (hz):");
    frequency_label->setToolTip("(Optional) frequency to run I/O at, default 1500 Hz. If set too high, system will not be able to keep up and I/O errors may trigger.");
    form->addRow(frequency_label, frequency_input);

    QLabel* watchdog_label = new QLabel("Watchdog timeout (ms):");
    watchdog_label->setToolTip("(Optional) Watchdog timeout for controllers on this I/O, default 10 ms. Do not increase unless you know what you are doing!");
    form->addRow(watchdog_label, watchdog_timeout_input);
}

void IOEditPage::initializePage() {
    if (config_id < 0) {
        setTitle("Add new I/O");
    } else {
        setTitle("Editing I/O config");
    }

    name_input->setText(QString::fromStdString(config->name));
    protocol_selector->setCurrentIndex(config->protocol.id());
    QString foot_pedals_name = config->foot_pedals.has_value() ? QString::fromStdString(config->foot_pedals->string()) : "";
    foot_pedals_file->setText(foot_pedals_name);
    frequency_input->setValue(int(1.0/config->period_ms));
    watchdog_timeout_input->setValue(config->watchdog_timeout_ms);
    emit completeChanged();
}

bool IOEditPage::nameAlreadyUsed() const {
    std::string proposed_name = name_input->text().toStdString();

    for (int i = 0; i < this->model->io_configs->count(); i++) {
        // if editing existing config, make sure we can keep the same name
        if (i == config_id) { continue; }

        std::string name = this->model->io_configs->get(i).name;
        if (name == proposed_name) {
            return true;
        }
    }

    return false;
}

bool IOEditPage::isComplete() const {
    return !nameAlreadyUsed() &&
           name_input->hasAcceptableInput() &&
           foot_pedals_file->hasAcceptableInput();
}

IOEditor::IOEditor(SystemConfigModel& model, QWidget* parent) : QWizard(parent), model(&model), config("") {
    edit_page = new IOEditPage(config, model);
    setPage(0, edit_page);

    setWizardStyle(QWizard::ModernStyle);
    setOption(QWizard::NoBackButtonOnStartPage);
    setWindowTitle("I/O Config Editor");

    QObject::connect(this, &QDialog::accepted, this, &IOEditor::done);
}

void IOEditor::setId(int id) {
    this->config_id = id;
    edit_page->setId(id);

    if (config_id >= 0) {
        config = model->io_configs->get(config_id);
    } else {
        config = IOConfig("");
    }

}

void IOEditor::done() {
    if (config_id < 0) {
        model->io_configs->addItem(config);
    } else {
        model->io_configs->updateItem(config_id, config);
    }
}

}

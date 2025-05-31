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

IOEditor::IOEditor(SystemConfigModel& model, QWidget* parent) : QWizard(parent), model(&model) {
    page = new QWizardPage;
    page->setTitle("I/O Config Editor");

    setWizardStyle(QWizard::ModernStyle);
    setOption(QWizard::NoBackButtonOnStartPage);

    QFormLayout* form = new QFormLayout(page);

    name_input = new QLineEdit();

    port_selector = new QComboBox();
    port_selector->setModel(&port_model);

    protocol_selector = new QComboBox();
    protocol_selector->setModel(&protocol_model);

    frequency_input = new QSpinBox();
    frequency_input->setRange(1, 10000);
    frequency_input->setSingleStep(100);
    frequency_input->setValue(1500);

    watchdog_timeout_input = new QDoubleSpinBox();
    watchdog_timeout_input->setRange(0.0, 5000.0);
    watchdog_timeout_input->setSingleStep(1.0);
    watchdog_timeout_input->setValue(10.0);

    form->addRow("Name:", name_input);
    form->addRow("Port:", port_selector);
    form->addRow("Protocol:", protocol_selector);
    form->addRow("Frequency (hz):", frequency_input);
    form->addRow("Watchdog timeout (ms):", watchdog_timeout_input);

    addPage(page);
    setWindowTitle("I/O Config Editor");

    QObject::connect(this, &QDialog::accepted, this, &IOEditor::done);
}

void IOEditor::setId(int id) {
    this->id = id;

    if (id < 0) {
        page->setTitle("Add new I/O");

        name_input->setText("io");
        port_selector->setCurrentIndex(-1);
        protocol_selector->setCurrentIndex(IOProtocol(IOProtocol::Value::SEQUENTIAL_READ_SEQUENTIAL_WRITE).id());
        frequency_input->setValue(1500);
        watchdog_timeout_input->setValue(10.0);
    } else {
        page->setTitle("Editing I/O config");

        IOConfig io = model->io_configs.get(id);

        name_input->setText(QString::fromStdString(io.name));
        port_selector->setCurrentIndex(io.port.id());
        protocol_selector->setCurrentIndex(io.protocol.id());
        frequency_input->setValue(int(1.0/io.period_ms));
        watchdog_timeout_input->setValue(io.watchdog_timeout_ms);
    }
}

void IOEditor::done() {
    IOConfig config = IOConfig(name_input->text().toStdString());
    config.port = IOPort(static_cast<IOPort::Value>(port_selector->currentIndex()));
    config.protocol = IOProtocol(static_cast<IOProtocol::Value>(protocol_selector->currentIndex()));
    config.period_ms = 1.0 / frequency_input->value();
    config.watchdog_timeout_ms = watchdog_timeout_input->value();

    if (id < 0) {
        model->io_configs.addItem(config);
    } else {
        model->io_configs.updateItem(id, config);
    }
}

}

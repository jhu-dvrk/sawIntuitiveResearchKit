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

IOEditor::IOEditor(QWidget* parent) : QWidget(parent) {
    QFormLayout* form = new QFormLayout(this);

    port_selector = new QComboBox();
    port_selector->setModel(&port_model);

    protocol_selector = new QComboBox();
    protocol_selector->setModel(&protocol_model);

    QSpinBox* period_selector = new QSpinBox();
    period_selector->setRange(1.0, 10000.0);
    period_selector->setSingleStep(100.0);
    period_selector->setValue(1500.0);

    QDoubleSpinBox* watchdog_timeout = new QDoubleSpinBox();
    watchdog_timeout->setValue(50);

    form->addRow("&Port: ", port_selector);
    form->addRow("&Protocol: ", protocol_selector);
    form->addRow("&Period (hz): ", period_selector);
    form->addRow("&Watchdog timeout (ms): ", watchdog_timeout);
}

}

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

#ifndef SYSTEM_WIZARD_IO_EDITOR
#define SYSTEM_WIZARD_IO_EDITOR

#include <QtWidgets>

#include "config_model.hpp"
#include "enum_list_model.hpp"

namespace system_wizard {

class IOEditor : public QWizard {
    Q_OBJECT

public:
    IOEditor(SystemConfigModel* model, QWidget* parent = nullptr);

public slots:
    void setId(int id);

private:
    void done();

    SystemConfigModel* model;
    int id = -1;

    QLineEdit* name_input;

    QSpinBox* frequency_input;
    QDoubleSpinBox* watchdog_timeout_input;

    QComboBox* port_selector;
    EnumListModel<IOPort> port_model;

    QComboBox* protocol_selector;
    EnumListModel<IOProtocol> protocol_model;
};

}

#endif // SYSTEM_WIZARD_IO_EDITOR

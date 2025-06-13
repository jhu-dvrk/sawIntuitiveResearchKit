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

class IOEditPage : public QWizardPage {
    Q_OBJECT

public:
    IOEditPage(IOConfig& config, const SystemConfigModel& model, QWidget* parent = nullptr);

    int nextId() const override { return -1; }

    void initializePage() override;

    bool nameAlreadyUsed() const;
    bool portAlreadyUsed() const;
    bool isComplete() const override;

    void setId(int id) {  config_id = id; }

private:
    IOConfig* config;
    int config_id = -1;

    const SystemConfigModel* model;

    QLineEdit* name_input;
    QLabel* name_invalid_msg;

    QSpinBox* frequency_input;
    QDoubleSpinBox* watchdog_timeout_input;

    QLineEdit* foot_pedals_file;

    QComboBox* port_selector;
    EnumListModel<IOPort> port_model;

    QComboBox* protocol_selector;
    EnumListModel<IOProtocol> protocol_model;
};

class IOEditor : public QWizard {
    Q_OBJECT

public:
    IOEditor(SystemConfigModel& model, QWidget* parent = nullptr);

public slots:
    void setId(int id);

private:
    void done();

    IOEditPage* edit_page;

    SystemConfigModel* model;
    IOConfig config;
    int config_id = -1;
};

}

#endif // SYSTEM_WIZARD_IO_EDITOR

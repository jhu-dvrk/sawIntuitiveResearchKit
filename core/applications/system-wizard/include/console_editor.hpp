/*
  Author(s):  Brendan Burkhart
  Created on: 2025-06-14

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_CONSOLE_EDITOR
#define SYSTEM_WIZARD_CONSOLE_EDITOR

#include <QtWidgets>

#include "models/config_model.hpp"

#include "console_inputs_editor.hpp"
#include "teleop_editor.hpp"

namespace system_wizard {

class ConsoleEditor : public QWidget {
    Q_OBJECT

public:
    ConsoleEditor(ConsoleConfig& config, SystemConfigModel& model, QWidget* parent = nullptr);

    bool close();

signals:
    void nameChanged(std::string name);

private:
    ConsoleConfig* config;
    ConsoleInputsEditor* inputs_editor;
    TeleopEditor teleop_editor;

    QLineEdit* name_input;
};

class ConsolesContainer : public QTabWidget {
    Q_OBJECT

public:
    ConsolesContainer(SystemConfigModel& model, QWidget* parent = nullptr);

private:
    void addConsole();
    void removeConsole(int index);

    SystemConfigModel* model;
};

}

#endif // SYSTEM_WIZARD_CONSOLE_EDITOR

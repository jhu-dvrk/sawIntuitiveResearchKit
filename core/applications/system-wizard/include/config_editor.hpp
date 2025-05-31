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

#ifndef SYSTEM_WIZARD_CONFIG_EDITOR
#define SYSTEM_WIZARD_CONFIG_EDITOR

#include <QtWidgets>

#include "arm_editor.hpp"
#include "arm_view.hpp"
#include "config_model.hpp"
#include "config_sources.hpp"
#include "io_editor.hpp"
#include "io_view.hpp"
#include "list_view.hpp"

namespace system_wizard {

class ConfigEditor : public QWidget {
    Q_OBJECT

public:
    ConfigEditor(SystemConfigModel* model, ConfigSources* config_sources, QWidget* parent = nullptr);

private:
    SystemConfigModel* model;

    ArmEditor arm_editor;
    std::unique_ptr<ArmViewFactory> arm_factory;

    IOEditor io_editor;
    std::unique_ptr<IOViewFactory> io_factory;

    ListView* io_list;
    ListView* arm_list;
    ListView* teleop_list;
    ListView* console_list;
};

}

#endif // SYSTEM_WIZARD_CONFIG_EDITOR

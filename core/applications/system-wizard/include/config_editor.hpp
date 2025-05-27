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

#include "io_editor.hpp"
#include "config_sources.hpp"

namespace system_wizard {

class ConfigEditor : public QWidget {
    Q_OBJECT

public:
    ConfigEditor(QWidget* parent = nullptr);

public slots:
    void armsChanged();

private:
};

}

#endif // SYSTEM_WIZARD_CONFIG_EDITOR

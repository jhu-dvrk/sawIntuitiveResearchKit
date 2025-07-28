/*
  Author(s):  Brendan Burkhart
  Created on: 2025-06-08

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_EDITOR
#define SYSTEM_WIZARD_EDITOR

#include <QtWidgets>

#include "config_editor.hpp"
#include "config_sources.hpp"

namespace system_wizard {

class Editor : public QStackedWidget {
    Q_OBJECT

public:
    Editor(ConfigSources& config_sources, QWidget* parent = nullptr);

public slots:
    void newConfig();
    void openConfig();
    void openConfigFile(std::filesystem::path config_file);

    void save();
    void saveAs();

    void closeConfig(int index);

private:
    void createTab(std::unique_ptr<ConfigEditor>);

    QTabWidget* tabs;
    ConfigSources* config_sources;

    QShortcut* close_config_shortcut;
};

}

#endif // SYSTEM_WIZARD_EDITOR

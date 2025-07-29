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

// weird Qt bug, apparently <filesystem> must be included *after* Qt headers
// see https://bugreports.qt.io/browse/QTBUG-73263
#include <filesystem>
#include <optional>

#include "arm_editor.hpp"
#include "config_sources.hpp"
#include "console_editor.hpp"
#include "io_editor.hpp"
#include "list_view.hpp"
#include "models/config_model.hpp"
#include "system_launcher.hpp"

namespace system_wizard {

class ConfigEditor : public QWidget {
    Q_OBJECT

public:
    ConfigEditor(
        std::unique_ptr<SystemConfigModel> config_model,
        ConfigSources& config_sources,
        SystemLauncher& launcher,
        QWidget* parent = nullptr
    );
    static std::unique_ptr<ConfigEditor> open(std::filesystem::path config_file, ConfigSources& sources, SystemLauncher& launcher);

    std::optional<std::filesystem::path> savePath() const { return save_path; }
    bool changesSaved() const { return changes_saved; }

public slots:
    bool save();
    bool saveAs();
    bool close();

signals:
    void savePathChanged(QString path);
    void saveStateChanged(bool changes_saved);

private:
    void setSavePath(std::filesystem::path path);
    void updateLaunchButton();

    std::unique_ptr<SystemConfigModel> model;
    bool changes_saved;
    std::optional<std::filesystem::path> save_path;

    QLabel* path_display;
    QPushButton* launch_button;

    IOEditor io_editor;
    ArmEditor arm_editor;

    ListView* io_list;
    ListView* arm_list;
    ConsolesContainer* console;

    SystemLauncher* launcher;
};

}

#endif // SYSTEM_WIZARD_CONFIG_EDITOR

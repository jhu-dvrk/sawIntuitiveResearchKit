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
#include "arm_view.hpp"
#include "config_sources.hpp"
#include "io_editor.hpp"
#include "io_view.hpp"
#include "list_view.hpp"
#include "models/config_model.hpp"
#include "teleop_editor.hpp"
#include "teleop_view.hpp"

namespace system_wizard {

class ConfigEditor : public QWidget {
    Q_OBJECT

public:
    ConfigEditor(std::unique_ptr<SystemConfigModel> config_model, ConfigSources& config_sources, QWidget* parent = nullptr);
    static std::unique_ptr<ConfigEditor> open(std::filesystem::path config_file, ConfigSources& sources);

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

    std::unique_ptr<SystemConfigModel> model;
    bool changes_saved;
    std::optional<std::filesystem::path> save_path;

    QLabel* path_display;

    IOEditor io_editor;
    IOViewFactory io_factory;

    ArmEditor arm_editor;
    ArmViewFactory arm_factory;

    TeleopEditor teleop_editor;
    TeleopViewFactory teleop_factory;

    ListView* io_list;
    ListView* arm_list;
    ListView* teleop_list;
    ListView* console_list;
};

}

#endif // SYSTEM_WIZARD_CONFIG_EDITOR

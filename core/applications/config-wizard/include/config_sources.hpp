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

#ifndef CONFIG_WIZARD_CONFIG_SOURCES
#define CONFIG_WIZARD_CONFIG_SOURCES

#include <QtWidgets>
#include <QFileSystemModel>
#include <QLabel>

#include <filesystem>
#include <vector>

#include "models/config_model.hpp"
#include "models/list_model.hpp"

namespace config_wizard {

class ConfigSources : public QWidget {
    Q_OBJECT

public:
    class Arm {
    public:
        Arm(std::string name, ArmType type, std::string serial, std::filesystem::path arm_file, std::optional<std::filesystem::path> io_file, bool is_DQLA)
            : name(name), type(type), serial_number(serial), arm_file(arm_file), io_file(io_file), is_DQLA(is_DQLA) {}

        std::string name;
        ArmType type;
        std::string serial_number;
        std::filesystem::path arm_file;
        std::optional<std::filesystem::path> io_file;
        std::optional<std::filesystem::path> io_gripper_file;
        bool is_DQLA;
    };

    ConfigSources(QWidget* parent = nullptr);

    ListModelT<Arm>& getModel();

    std::optional<std::filesystem::path> dir();

public slots:
    void addSource(QDir directory);
    void loaded(const QString &path);

private:
    QFileSystemModel* model;
    QTreeView *view;

    QLabel* source_dir_display;

    std::vector<std::filesystem::path> sources;

    ListModelT<Arm> arm_list_model;
};

}

#endif // CONFIG_WIZARD_CONFIG_SOURCES

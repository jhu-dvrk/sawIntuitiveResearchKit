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

#ifndef SYSTEM_WIZARD_CONFIG_SOURCES
#define SYSTEM_WIZARD_CONFIG_SOURCES

#include "config_model.hpp"
#include "list_model.hpp"

#include <QtWidgets>
#include <QFileSystemModel>
#include <QLabel>

#include <filesystem>
#include <memory>
#include <vector>

namespace system_wizard {

class ConfigSources : public QWidget {
    Q_OBJECT

public:
    class Arm {
    public:
        Arm(std::string name, ArmType type, std::string serial, std::filesystem::path config_file)
            : name(name), type(type), serial_number(serial), config_file(config_file) {}

        std::string name;
        ArmType type;
        std::string serial_number;
        std::filesystem::path config_file;
    };

    ConfigSources(QWidget* parent = nullptr);

    ListModelT<Arm>& getModel();

public slots:
    void add_source(QDir directory);
    void loaded(const QString &path);

private:
    QFileSystemModel* model;
    QTreeView *view;

    QLabel* source_dir_display;

    std::vector<std::filesystem::path> sources;

    VectorList<Arm> arm_list_model;
};

}

#endif // SYSTEM_WIZARD_CONFIG_SOURCES

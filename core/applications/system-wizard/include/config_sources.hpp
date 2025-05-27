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
        Arm(std::string type, std::string serial) : type(type), serial_number(serial) {}

        std::string type;
        std::string serial_number;
    };

    ConfigSources(QWidget* parent = nullptr);

    std::shared_ptr<std::vector<Arm>> availableArms() const;

signals:
    void armsChanged();

public slots:
    void add_source(QDir directory);
    void loaded(const QString &path);

private:
    QFileSystemModel* model;
    QTreeView *view;

    QLabel* source_dir_display;

    std::vector<std::filesystem::path> sources;
    std::shared_ptr<std::vector<Arm>> arms;
};

}

#endif // SYSTEM_WIZARD_CONFIG_SOURCES

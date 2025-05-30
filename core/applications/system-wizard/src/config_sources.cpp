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

#include "config_sources.hpp"

#include <filesystem>
#include <regex>

#include <QTreeView>

namespace system_wizard {

ConfigSources::ConfigSources(QWidget* parent) : QWidget(parent) {
    model = new QFileSystemModel();
    view = new QTreeView(this);
    view->setModel(model);

    setSizePolicy(QSizePolicy::Policy::Maximum, QSizePolicy::Policy::Preferred);

    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* title = new QLabel("Config sources");
    source_dir_display = new QLabel("");
    source_dir_display->setWordWrap(true);
    layout->addWidget(title);
    layout->addWidget(source_dir_display);
    layout->addWidget(view);

    QDir pwd = QDir::current();
    add_source(pwd);

    // Hide all columns except file name (column 0)
    for (int column = 1; column < model->columnCount(); column++) {
        view->setColumnHidden(column, true);
    }

    QObject::connect(model, &QFileSystemModel::directoryLoaded, this, &ConfigSources::loaded);
}

ListModelT<ConfigSources::Arm>& ConfigSources::getModel() {
    return arm_list_model;
}

void ConfigSources::add_source(QDir directory) {
    source_dir_display->setText(directory.path());
    auto index = model->setRootPath(directory.path());
    view->setRootIndex(index);
}

void ConfigSources::loaded(const QString &path) {
    std::filesystem::path root(path.toStdString());

    std::vector<Arm> new_arms;

    for (auto const& entry : std::filesystem::directory_iterator{root}) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const std::string name = entry.path().stem().string();

        // Match one of e.g. PSM2, ECM, MTMR2, followed by -, followed by five or six digit serial
        const std::regex arm_regex("(PSM|MTM(L|R)|ECM)(\\d*)\\-(\\d{5,6})");
        std::smatch base_match;
        bool matched = std::regex_match(name, base_match, arm_regex);
        if (!matched) {
            continue;
        }

        // capture groups are:
        // 0: entire match
        // 1: PSM/MTM/ECM
        // 2: L/R if MTM
        // 3: arm number, e.g. 2 for PSM2
        // 4: arm serial, e.g. 12345 for MTML-12345

        std::string type = base_match[1].str();
        std::string serial = base_match[4].str();

        new_arms.push_back(Arm(type, serial, entry.path()));
    }

    arm_list_model.update(new_arms);
    emit armsChanged();
}

}
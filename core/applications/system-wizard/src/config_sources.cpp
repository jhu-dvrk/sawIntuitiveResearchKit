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

ConfigSources::ConfigSources(QWidget* parent) : QWidget(parent), arms(std::make_shared<std::vector<Arm>>()) {
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

std::shared_ptr<std::vector<ConfigSources::Arm>> ConfigSources::availableArms() const {
    return arms;
}

void ConfigSources::add_source(QDir directory) {
    source_dir_display->setText(directory.path());
    auto index = model->setRootPath(directory.path());
    view->setRootIndex(index);
}

void ConfigSources::loaded(const QString &path) {
    std::filesystem::path root(path.toStdString());

    std::shared_ptr<std::vector<Arm>> new_arms = std::make_shared<std::vector<Arm>>();

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

        std::ssub_match type_sub_match = base_match[1];
        std::ssub_match serial_sub_match = base_match[4];

        new_arms->push_back(Arm(type_sub_match.str(), serial_sub_match.str()));
    }

    arms = new_arms;
    emit armsChanged();
}

}
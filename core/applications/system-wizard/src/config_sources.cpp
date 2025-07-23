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
#include <optional>
#include <regex>

#include <cisstCommon/cmnXMLPath.h>
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

std::optional<ConfigSources::Arm> parse_arm(std::filesystem::path file_path) {
    std::string file_name = file_path.stem().string();

    // Match one of e.g. PSM2, ECM, MTMR2, followed by -, followed by five or six digit serial
    const std::regex arm_regex("(PSM|MTM(L|R)|ECM)(\\d*)\\-(\\d{5,6})");
    std::smatch base_match;
    bool matched = std::regex_match(file_name, base_match, arm_regex);
    if (!matched) {
        return {};
    }

    // capture groups are:
    // 0: entire match
    // 1: PSM/MTML/MTMR/ECM
    // 2: L/R if MTM (already included in capture group 1)
    // 3: arm number, e.g. 2 for PSM2 or empty for ECM
    // 4: arm serial, e.g. 12345 for MTML-12345

    ArmType::Value arm_type;
    std::string type_str = base_match[1].str();
    if (type_str == "PSM") {
        arm_type = ArmType::Value::PSM;
    } else if (type_str == "MTML" || type_str == "MTMR") {
        arm_type = ArmType::Value::MTM;
    } else if (type_str == "ECM") {
        arm_type = ArmType::Value::ECM;
    } else {
        Q_ASSERT(false);
        return {};
    }

    std::string serial = base_match[4].str();
    std::string name = base_match[1].str() + base_match[3].str();

    std::string io_config_name = "sawRobotIO1394-" + base_match[0].str() + ".xml";
    std::filesystem::path io_config_path = file_path.parent_path() / io_config_name;
    cmnXMLPath io_config;
    io_config.SetInputSource(io_config_path.string());
    std::string hardware_version;
    io_config.GetXMLValue("", "/Config/Robot/@HardwareVersion", hardware_version, "");
    bool is_DQLA = hardware_version == "DQLA";

    return ConfigSources::Arm(name, arm_type, serial, file_path, is_DQLA);
}

std::optional<ConfigSources::Arm> parse_suj(std::filesystem::path file_path) {
    std::string file_name = file_path.stem().string();

    // currently there is a hard-coded assumption in dvrk_system that the SUJ component is named "SUJ"
    std::string name = "SUJ";

    if (file_name == "suj-fixed") {
        return ConfigSources::Arm(name, ArmType::Value::SUJ_FIXED, "", file_path, false);
    } else if (file_name == "suj-si") {
        return ConfigSources::Arm(name, ArmType::Value::SUJ_SI, "", file_path, false);
    } else if (file_name.size() >= 3 && file_name.substr(0, 3) == "suj") {
        return ConfigSources::Arm(name, ArmType::Value::SUJ_CLASSIC, "", file_path, false);
    } else {
        return {};
    }
}

void ConfigSources::loaded(const QString &path) {
    std::filesystem::path root(path.toStdString());

    std::vector<Arm> new_arms;

    for (auto const& entry : std::filesystem::directory_iterator{root}) {
        if (!entry.is_regular_file()) {
            continue;
        }

        std::optional<Arm> arm = parse_arm(entry.path());
        if (arm.has_value()) {
            new_arms.push_back(arm.value());
            continue;
        }

        std::optional<Arm> suj = parse_suj(entry.path());
        if (suj.has_value()) {
            new_arms.push_back(suj.value());
            continue;
        }
    }

    arm_list_model.replace(std::move(new_arms));
}

}

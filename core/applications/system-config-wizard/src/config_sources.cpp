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

#include <json/json.h>
#include <QTreeView>

namespace config_wizard {

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
    addSource(pwd);

    // Hide all columns except file name (column 0)
    for (int column = 1; column < model->columnCount(); column++) {
        view->setColumnHidden(column, true);
    }

    QObject::connect(model, &QFileSystemModel::directoryLoaded, this, &ConfigSources::loaded);
    QObject::connect(view, &QTreeView::doubleClicked, this, [this](const QModelIndex& index) {
        QString path = this->model->filePath(index);
        std::filesystem::path config_path = path.toStdString();

        // check if selected item is a system config file (rather than directory, arm config, etc.)
        if (!config_path.has_filename()) { return; }
        bool is_system_file = config_path.filename().string().substr(0, 6) == "system";
        if (!is_system_file) { return; }

        emit this->systemConfigChosen(path);
    });
}

ListModelT<ConfigSources::Arm>& ConfigSources::getModel() {
    return arm_list_model;
}

void ConfigSources::addSource(QDir directory) {
    std::filesystem::path path = std::filesystem::canonical(directory.path().toStdString());
    source_dir_display->setText(QString::fromStdString(path.string()));
    sources.push_back(path);
    auto index = model->setRootPath(QString::fromStdString(path.string()));
    view->setRootIndex(index);
}

std::optional<ConfigSources::Arm> parseArm(std::filesystem::path file_path) {
    // Make sure we don't pick up old .xml configs
    if (file_path.extension() != ".json") {
        return {};
    }

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

    ControllerType controller_type = ControllerType::QLA;

    std::string io_config_name = "sawRobotIO1394-" + base_match[0].str() + ".json";
    std::filesystem::path io_config_path = file_path.parent_path() / io_config_name;
    if (std::filesystem::exists(io_config_path)) {
        std::ifstream json_stream;
        Json::Value json_config;
        Json::Reader json_reader;

        json_stream.open(io_config_path.c_str());
        if (!json_reader.parse(json_stream, json_config)) {
            std::cerr << "failed to parse configuration file \"" << io_config_path << "\"\n"
                      << json_reader.getFormattedErrorMessages();
        }
        json_stream.close();
        std::string hardware_version;
        hardware_version = json_config["robots"][0]["hardware_version"].asString();
        if (hardware_version == "DQLA") {
            controller_type = ControllerType::DQLA;
        } else if (hardware_version == "dRA1") {
            controller_type = ControllerType::DRAC;
        } else if (hardware_version == "QLA1") {
            controller_type = ControllerType::QLA;
        } else {
            std::cerr << "unknown controller hardware version \"" << hardware_version << "\" found in " << io_config_path.string() << std::endl;
            return {};
        }
    }

    std::string io_gripper_config_name = "sawRobotIO1394-" + base_match[1].str() + "-gripper-" + base_match[4].str() + ".json";
    std::filesystem::path io_gripper_config_path = file_path.parent_path() / io_gripper_config_name;

    auto arm = ConfigSources::Arm(name, arm_type, serial, file_path, io_config_path, controller_type);

    if (std::filesystem::exists(io_gripper_config_path)) {
        arm.io_gripper_file = io_gripper_config_path;
    }

    return arm;
}

std::optional<ConfigSources::Arm> parseSUJ(std::filesystem::path file_path) {
    // Make sure we don't pick up old .xml configs
    if (file_path.extension() != ".json") {
        return {};
    }

    std::string file_name = file_path.stem().string();

    // currently there is a hard-coded assumption in dvrk_system that the SUJ component is named "SUJ"
    std::string name = "SUJ";

    std::string io_config_name = "sawRobotIO1394-" + name + ".json";
    std::filesystem::path io_config_path = file_path.parent_path() / io_config_name;

    if (file_name == "suj-fixed") {
        return ConfigSources::Arm(name, ArmType::Value::SUJ_FIXED, "", file_path, {}, ControllerType::OTHER);
    } else if (file_name == "suj-si") {
        return ConfigSources::Arm(name, ArmType::Value::SUJ_SI, "", file_path, {}, ControllerType::OTHER);
    } else if (file_name.size() >= 3 && file_name.substr(0, 3) == "suj") {
        return ConfigSources::Arm(name, ArmType::Value::SUJ_CLASSIC, "", file_path, io_config_path, ControllerType::OTHER);
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

        std::optional<Arm> arm = parseArm(entry.path());
        if (arm.has_value()) {
            new_arms.push_back(arm.value());
            continue;
        }

        std::optional<Arm> suj = parseSUJ(entry.path());
        if (suj.has_value()) {
            new_arms.push_back(suj.value());
            continue;
        }
    }

    arm_list_model.replace(std::move(new_arms));
}

std::optional<std::filesystem::path> ConfigSources::dir() {
    if (sources.size() > 0) {
        return sources[sources.size() - 1];
    } else {
        return {};
    }
}

}

/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-27

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_CONFIG_MODEL
#define SYSTEM_WIZARD_CONFIG_MODEL

#include "list_model.hpp"

#include <QtCore>

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <json/json.h>

namespace system_wizard {

enum class ArmConfigType {
    NATIVE,
    ROS_ARM,
    HAPTIC_MTM,
    SOCKET_PSM
};

class ArmType {
public:
    enum class Value {
        MTM,
        PSM,
        ECM,
        SUJ_CLASSIC,
        SUJ_SI,
        SUJ_FIXED,
        MTM_DERIVED,
        MTM_GENERIC,
        PSM_DERIVED,
        PSM_GENERIC,
        PSM_SOCKET,
        ECM_DERIVED,
        ECM_GENERIC,
        FOCUS_CONTROLLER,
    };

    ArmType(Value value) : value(value) {}

    static int count() { return 14; }

    int id() const { return static_cast<int>(value); }

    std::string name() const {
        switch (value) {
        case Value::MTM:
            return "MTM";
        case Value::PSM:
            return "PSM";
        case Value::ECM:
            return "ECM";
        case Value::SUJ_CLASSIC:
            return "SUJ Classic";
        case Value::SUJ_SI:
            return "SUJ Si";
        case Value::SUJ_FIXED:
            return "Fixed SUJ";
        case Value::MTM_DERIVED:
            return "Derived MTM";
        case Value::MTM_GENERIC:
            return "Generic MTM";
        case Value::PSM_DERIVED:
            return "Derived PSM";
        case Value::PSM_GENERIC:
            return "Generic PSM";
        case Value::PSM_SOCKET:
            return "Socket PSM";
        case Value::ECM_DERIVED:
            return "Derived ECM";
        case Value::ECM_GENERIC:
            return "Generic ECM";
        case Value::FOCUS_CONTROLLER:
            return "Focus controller";
        default:
            return "UNKNOWN";
        }
    }

    std::string acronym_expansion() const {
        switch (value) {
        case Value::MTM:
        case Value::MTM_DERIVED:
        case Value::MTM_GENERIC:
            return "Master Tool Manipulator";
        case Value::PSM:
        case Value::PSM_DERIVED:
        case Value::PSM_GENERIC:
        case Value::PSM_SOCKET:
            return "Patient Side Manipulator";
        case Value::ECM:
        case Value::ECM_DERIVED:
        case Value::ECM_GENERIC:
            return "Endoscopic Camera Manipulator";
        case Value::SUJ_CLASSIC:
        case Value::SUJ_SI:
        case Value::SUJ_FIXED:
            return "Set Up Joints";
        case Value::FOCUS_CONTROLLER:
            return "Focus controller";
        default:
            return "UNKNOWN";
        }
    }

    std::string explain() const {
        switch (value) {
        case Value::MTM:
        case Value::PSM:
        case Value::ECM:
            return "normal arm for dVRK harware";
        case Value::SUJ_CLASSIC:
            return "from Classic generation";
        case Value::SUJ_SI:
            return "from Si generation";
        case Value::SUJ_FIXED:
            return "virtual, using fixed transforms";
        case Value::MTM_DERIVED:
        case Value::PSM_DERIVED:
        case Value::ECM_DERIVED:
            return "for customizing behavior, based on existing code";
        case Value::MTM_GENERIC:
        case Value::PSM_GENERIC:
        case Value::ECM_GENERIC:
            return "for completely new arm code, e.g. a simulation";
        case Value::PSM_SOCKET:
            return "client for remote PSM via UDP socket (add socket server)";
        case Value::FOCUS_CONTROLLER:
            return "camera focus controller";
        default:
            return "UNKNOWN";
        }
    };

    static std::optional<ArmType> deserialize(std::string name) {
        if (name == "MTM") {
            return Value::MTM;
        } else if (name == "PSM") {
            return Value::PSM;
        } else if (name == "ECM") {
            return Value::ECM;
        } else if (name == "SUJ_Classic") {
            return Value::SUJ_CLASSIC;
        } else if (name == "SUJ_Si") {
            return Value::SUJ_SI;
        } else if (name == "SUJ_FIXED") {
            return Value::SUJ_FIXED;
        } else if (name == "MTM_DERIVED") {
            return Value::MTM_DERIVED;
        } else if (name == "MTM_GENERIC") {
            return Value::MTM_GENERIC;
        } else if (name == "PSM_DERIVED") {
            return Value::PSM_DERIVED;
        } else if (name == "PSM_GENERIC") {
            return Value::PSM_GENERIC;
        } else if (name == "PSM_SOCKET") {
            return Value::PSM_SOCKET;
        } else if (name == "ECM_DERIVED") {
            return Value::ECM_DERIVED;
        } else if (name == "ECM_GENERIC") {
            return Value::ECM_GENERIC;
        } else if (name == "FOCUS_CONTROLLER") {
            return Value::FOCUS_CONTROLLER;
        } else {
            return {};
        }
    }

    std::string serialize() const {
        switch (value) {
        case Value::MTM:
            return "MTM";
        case Value::PSM:
            return "PSM";
        case Value::ECM:
            return "ECM";
        case Value::SUJ_CLASSIC:
            return "SUJ_Classic";
        case Value::SUJ_SI:
            return "SUJ_Si";
        case Value::SUJ_FIXED:
            return "SUJ_FIXED";
        case Value::MTM_DERIVED:
            return "MTM_DERIVED";
        case Value::MTM_GENERIC:
            return "MTM_GENERIC";
        case Value::PSM_DERIVED:
            return "PSM_DERIVED";
        case Value::PSM_GENERIC:
            return "PSM_GENERIC";
        case Value::PSM_SOCKET:
            return "PSM_SOCKET";
        case Value::ECM_DERIVED:
            return "ECM_DERIVED";
        case Value::ECM_GENERIC:
            return "ECM_GENERIC";
        case Value::FOCUS_CONTROLLER:
            return "FOCUS_CONTROLLER";
        default:
            return "UNKNOWN";
        }
    };

    bool isPSM() const {
        switch (value) {
        case Value::PSM:
        case Value::PSM_DERIVED:
        case Value::PSM_GENERIC:
        case Value::PSM_SOCKET:
            return true;
        default:
            return false;
        }
    }

    bool isECM() const {
        switch (value) {
        case Value::ECM:
        case Value::ECM_DERIVED:
        case Value::ECM_GENERIC:
            return true;
        default:
            return false;
        }
    }

    bool isMTM() const {
        switch (value) {
        case Value::MTM:
        case Value::MTM_DERIVED:
        case Value::MTM_GENERIC:
            return true;
        default:
            return false;
        }
    }

    bool isSUJ() const {
        switch (value) {
        case Value::SUJ_CLASSIC:
        case Value::SUJ_SI:
        case Value::SUJ_FIXED:
            return true;
        default:
            return false;
        }
    }

    friend constexpr bool operator==(const ArmType& lhs, const ArmType& rhs) {
        return lhs.value == rhs.value;
    }
    friend constexpr bool operator!=(const ArmType& lhs, const ArmType& rhs) {
        return !(lhs == rhs);
    }

private:
    Value value;
};

class IOPort {
public:
    enum class Value {
        UDP = 0,
        FIREWIRE = 1,
        UDPFW = 2,
    };

    IOPort(Value value) : value(value) {}

    static int count() { return 3; }

    int id() const { return static_cast<int>(value); }

    std::string name() const {
        switch (value) {
        case Value::UDP:
            return "UDP";
        case Value::FIREWIRE:
            return "Firewire";
        case Value::UDPFW:
            return "UDP/Firewire";
        default:
            return "UNKNOWN";
        }
    };

    std::string explain() const {
        switch (value) {
        case Value::UDP:
            return "for all-Ethernet connections";
        case Value::FIREWIRE:
            return "for all-Firewire connections";
        case Value::UDPFW:
            return "for Firewire chains, but with Ethernet to PC";
        default:
            return "UNKNOWN";
        }
    };

    static std::optional<IOPort> deserialize(std::string name) {
        auto starts_with = [&name](std::string prefix) {
            return name.size() >= prefix.size() && name.substr(0, prefix.size()) == prefix;
        };

        if (starts_with("udp")) {
            if (starts_with("udpfw")) {
                return Value::UDPFW;
            } else {
                return Value::UDP;
            }
        } else if (starts_with("fw")) {
            return Value::FIREWIRE;
        } else {
            return {};
        }
    };

    std::string serialize() const {
        switch (value) {
        case Value::UDP:
            return "udp";
        case Value::FIREWIRE:
            return "fw";
        case Value::UDPFW:
            return "udpfw";
        default:
            return "UNKNOWN";
        }
    };

private:
    Value value;
};

class IOProtocol {
public:
    enum class Value {
        SEQUENTIAL_READ_SEQUENTIAL_WRITE = 0,
        SEQUENTIAL_READ_BROADCAST_WRITE = 1,
        BROADCAST_READ_BROADCAST_WRITE = 2,
    };

    IOProtocol(Value value) : value(value) {}

    static int count() { return 3; }

    int id() const { return static_cast<int>(value); }

    std::string name() const {
        switch (value) {
        case Value::SEQUENTIAL_READ_SEQUENTIAL_WRITE:
            return "Sequential read/sequential write";
        case Value::SEQUENTIAL_READ_BROADCAST_WRITE:
            return "Sequential read/broadcast write";
        case Value::BROADCAST_READ_BROADCAST_WRITE:
            return "Broadcast read/broadcast write";
        default:
            return "UNKNOWN";
        }
    };

    std::string explain() const {
        switch (value) {
        case Value::SEQUENTIAL_READ_SEQUENTIAL_WRITE:
        case Value::SEQUENTIAL_READ_BROADCAST_WRITE:
        case Value::BROADCAST_READ_BROADCAST_WRITE:
            return "Broadcast is usually faster, but is not supported by all hardware";
        default:
            return "UNKNOWN";
        }
    };

    static std::optional<IOProtocol> deserialize(std::string name) {
        if ((name == "sequential-read-write") ||
            (name == "srw")) {
            return Value::SEQUENTIAL_READ_SEQUENTIAL_WRITE;
        } else if ((name == "sequential-read-broadcast-write") ||
                   (name == "srbw")) {
            return Value::SEQUENTIAL_READ_BROADCAST_WRITE;
        } else if ((name == "broadcast-read-write") ||
                   (name == "brw") ||
                   (name == "broadcast-query-read-write") ||
                   (name == "bqrw")) {
            return Value::BROADCAST_READ_BROADCAST_WRITE;
        } else {
            return {};
        }
    };

    std::string serialize() const {
        switch (value) {
        case Value::SEQUENTIAL_READ_SEQUENTIAL_WRITE:
            return "sequential-read-write";
        case Value::SEQUENTIAL_READ_BROADCAST_WRITE:
            return "sequential-read-broadcast-write";
        case Value::BROADCAST_READ_BROADCAST_WRITE:
            return "broadcast-read-write";
        default:
            return "UNKNOWN";
        }
    };

private:
    Value value;
};

class TeleopType {
public:
    enum class Value {
        PSM_TELEOP,
        PSM_TELEOP_DERIVED,
        ECM_TELEOP,
        ECM_TELEOP_DERIVED
    };

    TeleopType(Value value) : value(value) {}

    static int count() { return 4; }

    int id() const { return static_cast<int>(value); }

    std::string name() const {
        switch (value) {
        case Value::PSM_TELEOP:
        case Value::PSM_TELEOP_DERIVED:
            return "PSM Teleop";
        case Value::ECM_TELEOP:
        case Value::ECM_TELEOP_DERIVED:
            return "Camera Teleop";
        default:
            return "UNKNOWN";
        }
    }

    std::string acronym_expansion() const {
        switch (value) {
        case Value::PSM_TELEOP:
        case Value::PSM_TELEOP_DERIVED:
            return "Teleoperation for Patient Side Manipulator";
        case Value::ECM_TELEOP:
        case Value::ECM_TELEOP_DERIVED:
            return "Teleoperation for Endoscopic Camera Manipulator";
        default:
            return "UNKNOWN";
        }
    }

    std::string explain() const {
        switch (value) {
        case Value::PSM_TELEOP:
        case Value::PSM_TELEOP_DERIVED:
            return "Control a PSM via an MTM input device";
        case Value::ECM_TELEOP:
        case Value::ECM_TELEOP_DERIVED:
            return "Control the endoscope camera using two MTM input devices";
        default:
            return "UNKNOWN";
        }
    };

    std::string serialize() const {
        switch (value) {
        case Value::PSM_TELEOP:
            return "PSM_TELEOP";
        case Value::PSM_TELEOP_DERIVED:
            return "PSM_TELEOP_DERIVED";
        case Value::ECM_TELEOP:
            return "ECM_TELEOP";
        case Value::ECM_TELEOP_DERIVED:
            return "ECM_TELEOP_DERIVED";
        default:
            return "UNKNOWN";
        }
    };

    friend constexpr bool operator==(const TeleopType& lhs, const TeleopType& rhs) {
        return lhs.value == rhs.value;
    }
    friend constexpr bool operator!=(const TeleopType& lhs, const TeleopType& rhs) {
        return !(lhs == rhs);
    }

private:
    Value value;
};

class IOConfig {
public:
    IOConfig(std::string name)
        : name(name),
          port(IOPort::Value::FIREWIRE),
          protocol(IOProtocol::Value::SEQUENTIAL_READ_SEQUENTIAL_WRITE),
          period_ms(1.0 / 1500.0),
          watchdog_timeout_ms(30.0)
        { }

    static IOConfig fromJSON(Json::Value value) {
        IOConfig config("io");
        if (value.isMember("footpedals")) {
            config.foot_pedals = value["footpedals"].asString();
        }
        auto port = IOPort::deserialize(value["port"].asString());
        if (port.has_value()) { config.port = port.value(); }
        auto protocol = IOProtocol::deserialize(value["firewire-protocol"].asString());
        if (protocol.has_value()) { config.protocol = protocol.value(); }

        config.period_ms = value.get("period", Json::Value(1.0/1500)).asDouble();
        config.watchdog_timeout_ms = value.get("watchdog-timeout", Json::Value(30.0)).asDouble();

        return config;
    }

    Json::Value toJSON() const {
        Json::Value value;

        if (foot_pedals) {
            value["footpedals"] = foot_pedals.value().string();
        }

        value["port"] = port.serialize();
        value["firewire-protocol"] = protocol.serialize();
        value["period"] = period_ms;
        value["watchdog-timeout"] = watchdog_timeout_ms;

        return value;
    }

    std::string name;
    std::optional<std::filesystem::path> foot_pedals;
    IOPort port;
    IOProtocol protocol;
    double period_ms;
    double watchdog_timeout_ms;
};

class ArmConfig {
public:
    ArmConfig(std::string name, ArmType type, ArmConfigType config_type)
    : name(name), type(type), config_type(config_type) { }

    static ArmConfig fromJSON(Json::Value value) {
        std::string name = value["name"].asString();
        ArmType type = ArmType::deserialize(value["type"].asString()).value();
        ArmConfigType config_type = ArmConfigType::NATIVE;

        ArmConfig config(name, type, config_type);
        config.serial_number = value["serial"].asString();

        return config;
    }

    Json::Value toJSON() const {
        Json::Value value;
        value["name"] = name;
        value["type"] = type.serialize();

        if (serial_number) {
            value["serial"] = serial_number.value();
        }

        return value;
    }

    std::string name;
    ArmType type;

    ArmConfigType config_type;
    std::optional<int> haptic_device;

    std::optional<std::string> serial_number;
    std::optional<bool> skip_ros_bridge;

    std::optional<std::string> component_name; // if not provided, uses Arm::name
    std::string interface_name;
};

class TeleopConfig {
public:
    TeleopConfig(std::string name, TeleopType type) : name(name), type(type) { }

    static TeleopConfig fromJSON(Json::Value value) {
        if (value.isMember("ecm")) {
            TeleopConfig config("ECM Teleop", TeleopType::Value::ECM_TELEOP);
            config.arm_names = {
                value["ecm"].asString(),
                value["mtm-left"].asString(),
                value["mtm-right"].asString()
            };
            return config;
        } else {
            std::vector<std::string> arm_names = {
                value["mtm"].asString(),
                value["psm"].asString()
            };
            std::string name = arm_names[0] + "-" + arm_names[1] + " Teleop";
            TeleopConfig config(name, TeleopType::Value::PSM_TELEOP);
            config.arm_names = arm_names;
            return config;
        }
    }

    Json::Value toJSON() const {
        Json::Value value;

        if (type == TeleopType::Value::PSM_TELEOP) {
            value["mtm"] = arm_names.at(0);
            value["psm"] = arm_names.at(1);
        } else if (type == TeleopType::Value::ECM_TELEOP) {
            value["ecm"] = arm_names.at(0);
            value["mtm-left"] = arm_names.at(1);
            value["mtm-right"] = arm_names.at(2);
        }

        return value;
    }

    std::string name;
    TeleopType type;

    std::vector<std::string> arm_names;
};

class ConsoleConfig {
public:
    static ConsoleConfig fromJSON(Json::Value value) {
        return ConsoleConfig();
    }

    Json::Value toJSON() const {
        return Json::Value();
    }
};

class SystemConfigModel : public QObject {
    Q_OBJECT

public:
    SystemConfigModel() : QObject() {
        io_configs = std::make_unique<ListModelT<IOConfig>>();
        arm_configs = std::make_unique<ListModelT<ArmConfig>>();
        teleop_configs = std::make_unique<ListModelT<TeleopConfig>>();
        console_configs = std::make_unique<ListModelT<ConsoleConfig>>();

        QObject::connect(io_configs.get(),      &ListModelT<IOConfig>::updated,      this, &SystemConfigModel::updated);
        QObject::connect(arm_configs.get(),     &ListModelT<ArmConfig>::updated,     this, &SystemConfigModel::updated);
        QObject::connect(teleop_configs.get(),  &ListModelT<TeleopConfig>::updated,  this, &SystemConfigModel::updated);
        QObject::connect(console_configs.get(), &ListModelT<ConsoleConfig>::updated, this, &SystemConfigModel::updated);
    }

    static std::unique_ptr<SystemConfigModel> load(std::filesystem::path config_file) {
        auto model = std::make_unique<SystemConfigModel>();

        std::ifstream json_stream(config_file, std::ios::in);

        Json::Value json_config, json_value;
        Json::Reader json_reader;
        if (!json_reader.parse(json_stream, json_config)) {
            qDebug() << QString::fromStdString(std::string(json_reader.getFormattedErrorMessages())) << "\n";
            return nullptr;
        }

        json_value = json_config["io"];
        if (!json_value.empty()) {
            IOConfig io_config = IOConfig::fromJSON(json_value);
            model->io_configs->addItem(io_config);
        }

        json_value = json_config["arms"];
        if (!json_value.empty() && json_value.isArray()) {
            model->arm_configs = ListModelT<ArmConfig>::fromJSON(json_value);
            if (model->arm_configs == nullptr) {
                return nullptr;
            }
        }

        json_value = json_config["psm-teleops"];
        if (!json_value.empty() && json_value.isArray()) {
            model->teleop_configs = ListModelT<TeleopConfig>::fromJSON(json_value);
            if (model->teleop_configs == nullptr) {
                return nullptr;
            }
        }

        json_value = json_config["ecm-teleop"];
        if (!json_value.empty() && json_value.isObject()) {
            TeleopConfig ecm_teleop = TeleopConfig::fromJSON(json_value);
            model->teleop_configs->addItem(ecm_teleop);
        }

        return model;
    }

    bool save(std::filesystem::path config_file) const {
        Json::Value config;

        if (io_configs->count() > 0) {
            config["io"] = io_configs->get(0).toJSON();
        }

        config["arms"] = arm_configs->toJSON();

        Json::Value psm_teleops(Json::arrayValue);
        for (int i = 0; i < teleop_configs->count(); i++) {
            TeleopConfig teleop_config = teleop_configs->get(i);
            if (teleop_config.type == TeleopType::Value::PSM_TELEOP) {
                psm_teleops.append(teleop_config.toJSON());
            }
        }
        config["psm-teleops"] = psm_teleops;

        for (int i = 0; i < teleop_configs->count(); i++) {
            qDebug() << i << ", " << teleop_configs->count() << "\n";
            TeleopConfig teleop_config = teleop_configs->get(i);
            if (teleop_config.type == TeleopType::Value::ECM_TELEOP) {
                config["ecm-teleop"] = teleop_config.toJSON();
                break;
            }
        }

        std::ofstream json_stream;
        json_stream.open(config_file.c_str());
        json_stream << config << std::endl;

        return true;
    }

signals:
    void updated();

public:
    std::unique_ptr<ListModelT<IOConfig>> io_configs;
    std::unique_ptr<ListModelT<ArmConfig>> arm_configs;
    std::unique_ptr<ListModelT<TeleopConfig>> teleop_configs;
    std::unique_ptr<ListModelT<ConsoleConfig>> console_configs;
};

}

#endif // SYSTEM_WIZARD_CONFIG_MODEL

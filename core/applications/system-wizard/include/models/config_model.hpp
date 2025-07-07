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

#include <QtCore>

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <qobjectdefs.h>
#include <string>
#include <vector>

#include <json/json.h>

#include "models/list_model.hpp"

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
    }

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
    }

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

    IOPort(Value value, std::optional<std::string> ip_address = {}, std::optional<int> firewire_port = {})
        : value(value), ip_address(ip_address), firewire_port(firewire_port) {}

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
    }

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
    }

    static std::optional<IOPort> deserialize(std::string name) {
        auto starts_with = [&name](std::string prefix) {
            return name.size() >= prefix.size() && name.substr(0, prefix.size()) == prefix;
        };

        Value type;

        if (starts_with("udp")) {
            if (starts_with("udpfw")) {
                type = Value::UDPFW;
            } else {
                type = Value::UDP;
            }
        } else if (starts_with("fw")) {
            type = Value::FIREWIRE;
        } else {
            return {}; // unknown/invalid
        }

        IOPort port = IOPort(type);

        std::string::size_type separator = name.find(":");
        if (separator == name.npos) {
            return port;
        }

        std::string remainder = name.substr(separator + 1, name.npos);
        if (type == Value::UDPFW || type == Value::UDP) {
            port.ip_address = remainder;
        } else {
            try {
                port.firewire_port = std::stoi(remainder);
            }
            catch (std::invalid_argument const& ex) { }
            catch (std::out_of_range const& ex) { }
        }

        return port;
    }

    std::string serialize() const {
        switch (value) {
        case Value::UDP:
            if (ip_address) {
                return "udp:" + ip_address.value();
            } else {
                return "udp";
            }
        case Value::FIREWIRE:
            if (firewire_port) {
                return "fw:" + std::to_string(firewire_port.value());
            } else {
                return "fw";
            }
        case Value::UDPFW:
            if (ip_address) {
                return "udpfw:" + ip_address.value();
            } else {
                return "udpfw";
            }
        default:
            return "UNKNOWN";
        }
    }

    bool conflicts(const IOPort& other) const {
        if (value == Value::UDP || value == Value::UDPFW) {
            bool also_udp = other.value == Value::UDP || other.value == Value::UDPFW;
            return also_udp && resolve_default_ip() == other.resolve_default_ip();
        } else {
            bool also_firewire = other.value == Value::FIREWIRE;
            bool same_port = firewire_port.value_or(0) == other.firewire_port.value_or(0);
            return also_firewire && same_port;
        }
    }

private:
    std::string resolve_default_ip() const {
        if (!ip_address.has_value() || ip_address.value().size() == 0) {
            return "169.254.0.100";
        } else {
            return ip_address.value();
        }
    }

    Value value;
    std::optional<std::string> ip_address;
    std::optional<int> firewire_port;
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
    }

    std::string explain() const {
        switch (value) {
        case Value::SEQUENTIAL_READ_SEQUENTIAL_WRITE:
        case Value::SEQUENTIAL_READ_BROADCAST_WRITE:
        case Value::BROADCAST_READ_BROADCAST_WRITE:
            return "Broadcast is usually faster, but is not supported by all hardware";
        default:
            return "UNKNOWN";
        }
    }

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
    }

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
    }

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
    }

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
    }

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

    static std::unique_ptr<IOConfig> fromJSON(Json::Value value) {
        auto config = std::make_unique<IOConfig>("io");
        if (value.isMember("footpedals")) {
            config->foot_pedals = value["footpedals"].asString();
        }
        auto port = IOPort::deserialize(value["port"].asString());
        if (port.has_value()) { config->port = port.value(); }
        auto protocol = IOProtocol::deserialize(value["firewire-protocol"].asString());
        if (protocol.has_value()) { config->protocol = protocol.value(); }

        config->period_ms = value.get("period", Json::Value(1.0/1500)).asDouble();

        if (value.isMember("watchdog-timeoue")) {
            config->watchdog_timeout_ms = value.get("watchdog-timeout", Json::Value(30.0)).asDouble();
        }

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

        if (watchdog_timeout_ms) {
            value["watchdog-timeout"] = watchdog_timeout_ms.value();
        }

        return value;
    }

    std::string name;
    std::optional<std::filesystem::path> foot_pedals;
    IOPort port;
    IOProtocol protocol;
    double period_ms;
    std::optional<double> watchdog_timeout_ms;
};

class ArmConfig {
public:
    ArmConfig(std::string name, ArmType type, ArmConfigType config_type)
    : name(name), type(type), config_type(config_type) { }

    static std::unique_ptr<ArmConfig> fromJSON(Json::Value value) {
        std::string name = value["name"].asString();
        ArmType type = ArmType::deserialize(value["type"].asString()).value();
        ArmConfigType config_type = ArmConfigType::NATIVE;

        auto config = std::make_unique<ArmConfig>(name, type, config_type);

        if (value.isMember("serial")) {
            config->serial_number = value["serial"].asString();
        }

        if (value.isMember("IO")) {
            config->io_name = value["IO"].asString();
        }

        return config;
    }

    Json::Value toJSON() const {
        Json::Value value;
        value["name"] = name;
        value["type"] = type.serialize();

        if (serial_number) {
            value["serial"] = serial_number.value();
        }

        if (io_name) {
            value["IO"] = io_name.value();
        }

        return value;
    }

    std::string name;
    ArmType type;

    ArmConfigType config_type;
    std::optional<int> haptic_device;

    std::optional<std::string> io_name;

    std::optional<std::string> serial_number;
    std::optional<bool> skip_ros_bridge;

    std::optional<std::string> component_name; // if not provided, uses Arm::name
    std::string interface_name;
};

class TeleopConfig {
public:
    TeleopConfig(std::string name, TeleopType type) : name(name), type(type) { }

    static std::unique_ptr<TeleopConfig> fromJSON(Json::Value value) {
        if (value.isMember("ecm")) {
            auto config = std::make_unique<TeleopConfig>("ECM Teleop", TeleopType::Value::ECM_TELEOP);
            config->arm_names = {
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
            auto config = std::make_unique<TeleopConfig>(name, TeleopType::Value::PSM_TELEOP);
            config->arm_names = arm_names;
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

class ConsoleInputType {
public:
    enum class Value {
        NONE,
        FOOT_PEDALS,
        FORCE_DIMENSION_BUTTONS,
        CUSTOM_BUTTONS
    };

    ConsoleInputType(Value value) : value(value) {}

    static int count() { return 4; }

    int id() const { return static_cast<int>(value); }

    std::string name() const {
        switch (value) {
        case Value::NONE:
            return "None";
        case Value::FOOT_PEDALS:
            return "Foot pedals";
        case Value::FORCE_DIMENSION_BUTTONS:
            return "ForceDimension inputs";
        case Value::CUSTOM_BUTTONS:
            return "Custom inputs";
        default:
            return "UNKNOWN";
        }
    }

    std::string explain() const {
        switch (value) {
        case Value::NONE:
            return "No user inputs";
        case Value::FOOT_PEDALS:
            return "User input from surgeon console foot pedals";
        case Value::FORCE_DIMENSION_BUTTONS:
            return "User input from buttons on ForceDimension haptic device";
        case Value::CUSTOM_BUTTONS:
            return "Custom user inputs";
        default:
            return "UNKNOWN";
        }
    }

    friend constexpr bool operator==(const ConsoleInputType& lhs, const ConsoleInputType& rhs) {
        return lhs.value == rhs.value;
    }
    friend constexpr bool operator!=(const ConsoleInputType& lhs, const ConsoleInputType& rhs) {
        return !(lhs == rhs);
    }

private:
    Value value;
};

class HeadSensorType {
public:
    enum class Value {
        NONE,
        GOOVIS,
        ISI,
        DVRK
    };

    HeadSensorType(Value value) : value(value) {}

    static int count() { return 4; }

    int id() const { return static_cast<int>(value); }

    std::string name() const {
        switch (value) {
        case Value::NONE:
            return "None";
        case Value::GOOVIS:
            return "Goovis HMD head sensor";
        case Value::ISI:
            return "Original ISI head sensor";
        case Value::DVRK:
            return "dVRK head sensor";
        default:
            return "UNKNOWN";
        }
    }

    std::string explain() const {
        switch (value) {
        case Value::NONE:
            return "No head sensor";
        case Value::GOOVIS:
            return "Built-in head sensor from Goovis HMD";
        case Value::ISI:
            return "Original head sensor from daVinci surgeon console";
        case Value::DVRK:
            return "Open-source retrofit dVRK head sensor";
        default:
            return "UNKNOWN";
        }
    }

    friend constexpr bool operator==(const HeadSensorType& lhs, const HeadSensorType& rhs) {
        return lhs.value == rhs.value;
    }
    friend constexpr bool operator!=(const HeadSensorType& lhs, const HeadSensorType& rhs) {
        return !(lhs == rhs);
    }

private:
    Value value;
};

class FootPedalsConfig : public QObject {
    Q_OBJECT

public:
    FootPedalsConfig() : QObject() {}

    std::string source_arm_name;
};

class HeadSensorConfig : public QObject {
    Q_OBJECT

public:
    HeadSensorConfig() : QObject(), head_sensor_type(HeadSensorType::Value::NONE) {}

    HeadSensorType head_sensor_type;
    std::string source_arm_name;
};

class ForceDimensionButtonInputConfig : public QObject {
    Q_OBJECT

public:
    ForceDimensionButtonInputConfig() : QObject() {}

    std::string source_arm_name;
};

class CustomButtonInputConfig : public QObject {
    Q_OBJECT

public:
    CustomButtonInputConfig() : QObject() {}

    std::string operator_present_component;
    std::string operator_present_interface;
    
    std::string clutch_component;
    std::string clutch_interface;
    
    std::string camera_component;
    std::string camera_interface;
};

class ConsoleInputConfig : public QObject {
    Q_OBJECT

public:
    ConsoleInputConfig() : QObject(), type(ConsoleInputType::Value::NONE) {}

    ConsoleInputType type;

    FootPedalsConfig pedals;
    HeadSensorConfig head_sensor;
    ForceDimensionButtonInputConfig force_dimension_buttons;
    CustomButtonInputConfig custom_buttons;
};

class ConsoleConfig : public QObject {
    Q_OBJECT

public:
    ConsoleConfig() : QObject() {
        psm_teleops = std::make_unique<ListModelT<TeleopConfig>>();
        ecm_teleops = std::make_unique<ListModelT<TeleopConfig>>();
    }

    static std::unique_ptr<ConsoleConfig> fromJSON(Json::Value json_config) {
        auto config = std::make_unique<ConsoleConfig>();
        Json::Value json_value;

        config->name = json_config["name"].asString();

        json_value = json_config["teleop_PSMs"];
        if (!json_value.empty() && json_value.isArray()) {
            config->psm_teleops = ListModelT<TeleopConfig>::fromJSON(json_value);
            if (config->psm_teleops == nullptr) {
                return {};
            }
        }

        json_value = json_config["teleop_ECMs"];
        if (!json_value.empty() && json_value.isArray()) {
            config->ecm_teleops = ListModelT<TeleopConfig>::fromJSON(json_value);
            if (config->ecm_teleops == nullptr) {
                return {};
            }
        }

        return config;
    }

    Json::Value toJSON() const {
        Json::Value value;

        value["name"] = name;
        value["teleop_PSMs"] = psm_teleops->toJSON();
        value["teleop_ECMs"] = ecm_teleops->toJSON();

        return value;
    }

    std::string name;

    ConsoleInputConfig inputs;

    std::unique_ptr<ListModelT<TeleopConfig>> psm_teleops;
    std::unique_ptr<ListModelT<TeleopConfig>> ecm_teleops;
};

class SystemConfigModel : public QObject {
    Q_OBJECT

    using ConsoleList_t = ListModelT<std::unique_ptr<ConsoleConfig>>;

public:
    SystemConfigModel() : QObject() {
        io_configs = std::make_unique<ListModelT<IOConfig>>();
        arm_configs = std::make_unique<ListModelT<ArmConfig>>();
        console_configs = std::make_unique<ConsoleList_t>();

        QObject::connect(io_configs.get(),      &ListModelT<IOConfig>::updated,  this, &SystemConfigModel::updated);
        QObject::connect(arm_configs.get(),     &ListModelT<ArmConfig>::updated, this, &SystemConfigModel::updated);
        QObject::connect(console_configs.get(), &ConsoleList_t::updated,         this, &SystemConfigModel::updated);
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

        json_value = json_config["IOs"];
        if (!json_value.empty() && json_value.isArray()) {
            model->io_configs = ListModelT<IOConfig>::fromJSON(json_value);
            if (model->io_configs == nullptr) {
                return nullptr;
            }
        }

        json_value = json_config["arms"];
        if (!json_value.empty() && json_value.isArray()) {
            model->arm_configs = ListModelT<ArmConfig>::fromJSON(json_value);
            if (model->arm_configs == nullptr) {
                return nullptr;
            }
        }

        json_value = json_config["consoles"];
        if (!json_value.empty() && json_value.isArray()) {
            model->console_configs = ConsoleList_t::fromJSON(json_value);
            if (model->console_configs == nullptr) {
                return nullptr;
            }
        }

        return model;
    }

    bool save(std::filesystem::path config_file) const {
        Json::Value config;

        config["IOs"] = io_configs->toJSON();
        config["arms"] = arm_configs->toJSON();
        config["consoles"] = console_configs->toJSON();

        std::ofstream json_stream;
        json_stream.open(config_file.c_str());
        json_stream << config << std::endl;

        return true;
    }

signals:
    void updated();

public:
    // need to be pointers to comply with Qt's "identity not value" philosophy
    std::unique_ptr<ListModelT<IOConfig>> io_configs;
    std::unique_ptr<ListModelT<ArmConfig>> arm_configs;
    std::unique_ptr<ConsoleList_t> console_configs;
};

}

#endif // SYSTEM_WIZARD_CONFIG_MODEL

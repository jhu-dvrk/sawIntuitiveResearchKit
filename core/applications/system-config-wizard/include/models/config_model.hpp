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

#ifndef CONFIG_WIZARD_CONFIG_MODEL
#define CONFIG_WIZARD_CONFIG_MODEL

#include <QtCore>

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <json/json.h>
#include "cisstVector/vctTransformationTypes.h"

#include "models/list_model.hpp"

namespace config_wizard {

class ComponentConfig {
public:
    static ComponentConfig fromJSON(Json::Value json) {
        auto config = ComponentConfig();
        config.library_name = json.get("shared-library", "").asString();
        config.class_name = json.get("class-name", "").asString();
        config.name = json["constructor-arg"].get("Name", "").asString();
        config.configure_parameter = json.get("configure-parameter", "").asString();
        if (json["constructor-arg"].isMember("Period")) {
            config.period = json["constructor-arg"].get("Period", 0.01).asDouble();
        }

        return config;
    }

    Json::Value toJSON(std::filesystem::path CMN_UNUSED(destination)) const {
        Json::Value value;

        value["shared-library"] = library_name;
        value["class-name"] = class_name;

        Json::Value constructor_args;
        constructor_args["Name"] = name;
        if (period.has_value()) {
            constructor_args["Period"] = period.value();
        }
        value["constructor-arg"] = constructor_args;

        if (!configure_parameter.empty()) {
            value["configure-parameter"] = configure_parameter;
        }

        return value;
    }

    std::string library_name;
    std::string class_name;
    std::string name;
    std::string configure_parameter;
    std::optional<double> period;
};

class ComponentInterfaceConfig {
public:
    static std::unique_ptr<ComponentInterfaceConfig> fromJSON(Json::Value json) {
        auto config = std::make_unique<ComponentInterfaceConfig>();
        config->component_name = json.get("component", "").asString();
        config->interface_name = json.get("interface", "").asString();
        return config;
    }

    Json::Value toJSON(std::filesystem::path CMN_UNUSED(destination)) const {
        Json::Value json;
        json["component"] = component_name;
        json["interface"] = interface_name;
        return json;
    }

    std::string component_name;
    std::string interface_name;

    std::optional<ComponentConfig> component;
};

enum class ArmConfigType {
    NATIVE,
    ROS_ARM,
    HAPTIC_MTM,
    SIMULATED
};

enum class ControllerType {
    QLA,
    DQLA,
    DRAC,
    OTHER
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
        } else if (name == "SUJ_Fixed") {
            return Value::SUJ_FIXED;
        } else if (name == "MTM_DERIVED") {
            return Value::MTM_DERIVED;
        } else if (name == "MTM_GENERIC") {
            return Value::MTM_GENERIC;
        } else if (name == "PSM_DERIVED") {
            return Value::PSM_DERIVED;
        } else if (name == "PSM_GENERIC") {
            return Value::PSM_GENERIC;
        }  else if (name == "ECM_DERIVED") {
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
            return "SUJ_Fixed";
        case Value::MTM_DERIVED:
            return "MTM_DERIVED";
        case Value::MTM_GENERIC:
            return "MTM_GENERIC";
        case Value::PSM_DERIVED:
            return "PSM_DERIVED";
        case Value::PSM_GENERIC:
            return "PSM_GENERIC";
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

    bool isPSM() const {
        return value == Value::PSM_TELEOP || value == Value::PSM_TELEOP_DERIVED;
    }

    bool isECM() const {
        return value == Value::ECM_TELEOP || value == Value::ECM_TELEOP_DERIVED;
    }

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
          protocol(IOProtocol::Value::BROADCAST_READ_BROADCAST_WRITE)
        { }

    static std::unique_ptr<IOConfig> fromJSON(Json::Value value) {
        auto config = std::make_unique<IOConfig>("io");
        config->name = value.get("name", "").asString();

        auto port = IOPort::deserialize(value["port"].asString());
        if (port.has_value()) { config->port = port.value(); }
        auto protocol = IOProtocol::deserialize(value["protocol"].asString());
        if (protocol.has_value()) { config->protocol = protocol.value(); }

        if (value.isMember("period")) {
            config->period_ms = value["period"].asDouble();
        }

        if (value.isMember("watchdog-timeoue")) {
            config->watchdog_timeout_ms = value["watchdog-timeout"].asDouble();
        }

        return config;
    }

    Json::Value toJSON(std::filesystem::path CMN_UNUSED(destination)) const {
        Json::Value value;

        value["name"] = name;
        value["port"] = port.serialize();
        value["protocol"] = protocol.serialize();

        if (period_ms) {
            value["period"] = period_ms.value();
        }

        if (watchdog_timeout_ms) {
            value["watchdog-timeout"] = watchdog_timeout_ms.value();
        }

        return value;
    }

    std::string name;
    IOPort port;
    IOProtocol protocol;
    std::optional<double> period_ms;
    std::optional<double> watchdog_timeout_ms;
};

class BaseFrameConfig {
public:
    BaseFrameConfig() {}

    static std::unique_ptr<BaseFrameConfig> fromJSON(Json::Value json) {
        auto config = std::make_unique<BaseFrameConfig>();
        config->use_custom_transform = json.isMember("reference_frame") && json.isMember("transform");

        if (config->use_custom_transform) {
            config->reference_frame_name = json["reference_frame"].asString();
            cmnDataJSON<vctFrm4x4>::DeSerializeText(config->transform, json["transform"]);
        } else {
            config->transform = decltype(transform)::Identity();
            auto base_frame = ComponentInterfaceConfig::fromJSON(json);
            if (base_frame != nullptr) {
                config->base_frame_component = *base_frame;
            }
        }

        return config;
    }

    Json::Value toJSON(std::filesystem::path destination) const {
        if (use_custom_transform) {
            Json::Value json;
            json["reference_frame"] = reference_frame_name;
            cmnDataJSON<vctFrm4x4>::SerializeText(transform, json["transform"]);
            return json;
        } else {
            return base_frame_component.toJSON(destination);
        }
    }

    bool use_custom_transform;
    std::string reference_frame_name;
    vctFrm4x4 transform;
    ComponentInterfaceConfig base_frame_component;
};

class ArmConfig {
public:
    ArmConfig(std::string name, ArmType type, ArmConfigType config_type) :
        name(name), type(type), config_type(config_type)
    { }

    static std::unique_ptr<ArmConfig> fromJSON(Json::Value value) {
        std::string name = value["name"].asString();
        auto type = ArmType::deserialize(value["type"].asString());
        if (!type.has_value()) {
            return nullptr;
        }

        ArmConfigType config_type = ArmConfigType::NATIVE;
        auto config = std::make_unique<ArmConfig>(name, type.value(), config_type);

        if (value.isMember("component")) {
            auto component = ComponentInterfaceConfig::fromJSON(value);
            if (component != nullptr) {
                config->component = *component;
            }
        }

        if (value.isMember("base_frame")) {
            auto base_frame = BaseFrameConfig::fromJSON(value["base_frame"]);
            if (base_frame != nullptr) {
                config->base_frame = *base_frame;
            }
        }

        if (value.isMember("serial")) {
            config->serial_number = value["serial"].asString();
        }

        if (value.isMember("simulation")) {
            // simulation options are DYNAMIC (unsupported/unused), NONE, or KINEMATIC
            bool is_kinematic_simulatation = value["simulation"].asString() == "KINEMATIC";
            config->is_simulated = is_kinematic_simulatation;
            config->config_type = ArmConfigType::SIMULATED;
        }

        if (value.isMember("IO")) {
            config->io_name = value["IO"].asString();
        }

        if (value.isMember("arm_file")) {
            config->arm_file = value["arm_file"].asString();
        }

        if (value.isMember("IO_file")) {
            config->io_file = value["IO_file"].asString();
        }

        if (value.isMember("IO_gripper_file")) {
            config->io_gripper_file = value["IO_gripper_file"].asString();
        }

        if (type->isSUJ()) {
            config->controller_type = ControllerType::OTHER;
        }

        return config;
    }

    void configureComponent(ComponentConfig config) {
        if (!component.has_value()) {
            component = ComponentInterfaceConfig();
            component->component_name = config.name;
            component->interface_name = name;
        }

        component->component = config;
        if (config.library_name == "sawForceDimensionSDK") {
            config_type = ArmConfigType::HAPTIC_MTM;
            haptic_device = 0;
        } else if (config.library_name == "dvrk_arm_from_ros") {
            config_type = ArmConfigType::ROS_ARM;
        }
    }

    Json::Value toJSON(std::filesystem::path destination) const {
        Json::Value value;
        value["name"] = name;
        value["type"] = type.serialize();

        if (arm_file.has_value()) {
            // only save arm config file path if it differs from the default
            std::string default_filename = name + "-" + serial_number.value_or("") + ".json";
            if (arm_file->filename() != default_filename || arm_file->parent_path() != destination.parent_path()) {
                value["arm_file"] = arm_file->string();
            }
        }

        if (io_file.has_value() && arm_file->parent_path() != destination.parent_path()) {
            // only save IO config file path if it differs from the default
            std::string default_filename = "sawRobotIO1394-" + name +  + "-" + serial_number.value_or("") + ".json";
            if (io_file->filename() != default_filename || io_file->parent_path() != destination.parent_path()) {
                value["IO_file"] = io_file->string();
            }
        }

        if (io_gripper_file.has_value()) {
            // only save IO gripper config file path if it differs from the default
            std::string default_filename = "sawRobotIO1394-" + name + "-gripper-" + serial_number.value_or("") + ".json";
            if (io_gripper_file->filename() != default_filename || io_gripper_file->parent_path() != destination.parent_path()) {
                value["IO_gripper_file"] = io_gripper_file->string();
            }
        }

        if (config_type == ArmConfigType::NATIVE || config_type == ArmConfigType::SIMULATED) {
            if (base_frame) {
                value["base_frame"] = base_frame->toJSON(destination);
            }

            if (is_simulated.has_value() && is_simulated.value()) {
                value["simulation"] = "KINEMATIC";
            } else {
                if (io_name) {
                    value["IO"] = io_name.value();
                }
                if (serial_number) {
                    value["serial"] = serial_number.value();
                }
            }
        } else if (config_type == ArmConfigType::HAPTIC_MTM) {
            if (component) {
                value["component"] = component->component_name;
                value["interface"] = component->interface_name;
            }
        } else if (config_type == ArmConfigType::ROS_ARM) {
            // Real/remote arm already publishes to ROS
            value["skip_ROS_bridge"] = true;

            if (component) {
                value["component"] = component->component_name;
                value["interface"] = component->interface_name;
            }
        }

        return value;
    }

    std::string name;
    ArmType type;

    std::optional<std::filesystem::path> arm_file;
    std::optional<std::filesystem::path> io_file;
    std::optional<std::filesystem::path> io_gripper_file;

    ArmConfigType config_type;
    std::optional<int> haptic_device;

    std::optional<std::string> io_name;
    std::optional<std::string> serial_number;
    std::optional<bool> is_simulated;
    ControllerType controller_type = ControllerType::QLA;

    std::optional<BaseFrameConfig> base_frame;
    std::optional<ComponentInterfaceConfig> component;
};

class TeleopConfig {
public:
    TeleopConfig(std::string name, TeleopType type) : name(name), type(type) { }

    static std::unique_ptr<TeleopConfig> fromJSON(Json::Value value) {
        std::unique_ptr<TeleopConfig> config;

        if (value.isMember("ECM")) {
            config = std::make_unique<TeleopConfig>("ECM Teleop", TeleopType::Value::ECM_TELEOP);
            config->arm_names = {
                value["ECM"].asString(),
                value["MTML"].asString(),
                value["MTMR"].asString()
            };
        } else {
            std::vector<std::string> arm_names = {
                value["MTM"].asString(),
                value["PSM"].asString()
            };
            std::string name = arm_names[0] + "-" + arm_names[1] + " Teleop";
            config = std::make_unique<TeleopConfig>(name, TeleopType::Value::PSM_TELEOP);
            config->arm_names = arm_names;

            if (value.isMember("PSM_base_frame")) {
                config->PSM_base_frame = *ComponentInterfaceConfig::fromJSON(value["PSM_base_frame"]);
            }
        }

        Json::Value parameters = value["configure_parameter"];
        if (parameters.isMember("scale")) {
            config->scale = parameters["scale"].asDouble();
        }

        bool has_wrist_actuatation = parameters.get("align_MTM", true).asBool();
        config->has_MTM_wrist_actuation = has_wrist_actuatation;

        bool has_gripper = !parameters.get("ignore_jaw", false).asBool();
        config->has_MTM_gripper = has_gripper;

        return config;
    }

    Json::Value toJSON(std::filesystem::path destination) const {
        Json::Value value;
        Json::Value parameters;

        if (type == TeleopType::Value::PSM_TELEOP) {
            value["MTM"] = arm_names.at(0);
            value["PSM"] = arm_names.at(1);

            if (PSM_base_frame.has_value()) {
                value["PSM_base_frame"] = PSM_base_frame->toJSON(destination);
            }
        } else if (type == TeleopType::Value::ECM_TELEOP) {
            value["ECM"] = arm_names.at(0);
            value["MTML"] = arm_names.at(1);
            value["MTMR"] = arm_names.at(2);
        }

        parameters["scale"] = scale;

        if (type.isPSM()) {
            if (!has_MTM_gripper) {
                parameters["start_gripper_threshold"] = 0.0;
                parameters["ignore_jaw"] = true;
            }
            if (!has_MTM_wrist_actuation) {
                parameters["align_MTM"] = false;
            }
        }
        value["configure_parameter"] = parameters;

        return value;
    }

    std::string name;
    TeleopType type;

    std::vector<std::string> arm_names;

    double scale = 0.5;
    bool has_MTM_gripper = false;
    bool has_MTM_wrist_actuation = false;
    std::optional<ComponentInterfaceConfig> PSM_base_frame;

    void provideSources(ListModelT<ArmConfig> const& arms) {
        this->arms = &arms;
    }

    std::optional<ArmConfig> getSource(std::string name) const {
        if (arms == nullptr) { return {}; }

        std::optional<ArmConfig> source = arms->find(
            [&name](const ArmConfig& arm){ return arm.name == name; }
        );

        return source;
    }

private:
    ListModelT<ArmConfig> const* arms = nullptr;
};

class ConsoleInputType {
public:
    enum class Value {
        NONE,
        FOOT_PEDALS,
        FORCE_DIMENSION_BUTTONS
    };

    ConsoleInputType(Value value) : value(value) {}

    static int count() { return 3; }

    int id() const { return static_cast<int>(value); }

    std::string name() const {
        switch (value) {
        case Value::NONE:
            return "None";
        case Value::FOOT_PEDALS:
            return "Foot pedals";
        case Value::FORCE_DIMENSION_BUTTONS:
            return "ForceDimension inputs";
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
            return "Open-source dVRK head sensor";
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
            return "Original ISI head sensor from surgeon console";
        case Value::DVRK:
            return "Open-source dVRK head sensor retrofit on to surgeon console";
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

    static std::unique_ptr<FootPedalsConfig> fromJSON(Json::Value value) {
        std::string io_file = value.get("IO_file", "").asString();
        if (io_file == "") { return nullptr; }

        std::string prefix = "sawRobotIO1394-";
        std::string suffix = "-foot-pedals";
        size_t start_idx = io_file.find(prefix) + prefix.size();
        size_t end_idx = io_file.find(suffix);

        if (end_idx == std::string::npos || start_idx == std::string::npos || end_idx <= start_idx) {
            return nullptr;
        }

        std::string arm_name = io_file.substr(start_idx, end_idx - start_idx);

        std::string dqla_suffix = "-DQLA.json";
        size_t n = dqla_suffix.size();
        bool is_dqla = io_file.size() >= n && io_file.substr(io_file.size() - n, n) == dqla_suffix;

        auto config = std::make_unique<FootPedalsConfig>();
        config->source_arm_name = arm_name;
        config->source_io_name = value.get("IO", "").asString();
        config->source_is_dqla = is_dqla;
        return config;
    }

    Json::Value toJSON(std::filesystem::path CMN_UNUSED(destination)) const {
        auto source = getSource();
        std::string io_name = source && source->io_name ? *source->io_name : source_io_name;
        bool is_dqla = source ? (source->controller_type == ControllerType::DQLA) : source_is_dqla;

        Json::Value value;
        value["IO"] = io_name;
        value["IO_file"] = "io/sawRobotIO1394-" + source_arm_name + "-foot-pedals" + (is_dqla ? "-DQLA" : "-QLA1") + ".json";

        return value;
    }

    void provideSources(ListModelT<ArmConfig> const& arms) {
        this->arms = &arms;
    }

    /* Store source arm name, and prefer retrieving IO name from current arm, so that changes to source arm config are reflected here */
    std::string source_arm_name;
    bool source_is_dqla = false;
    std::string source_io_name;

private:
    ListModelT<ArmConfig> const* arms = nullptr;

    std::optional<ArmConfig> getSource() const {
        if (arms == nullptr) { return {}; }

        std::optional<ArmConfig> source = arms->find(
            [this](const ArmConfig& arm){ return arm.name == source_arm_name; }
        );

        return source;
    }
};

class HeadSensorConfig : public QObject {
    Q_OBJECT

public:
    HeadSensorConfig(HeadSensorType type) : QObject(), type(type) {}

    static std::unique_ptr<HeadSensorConfig> fromJSON(Json::Value value) {
        std::string input_type = value.get("input_type", "").asString();

        if (input_type == "PEDALS_GOOVIS_HEAD_SENSOR") {
            return std::make_unique<HeadSensorConfig>(HeadSensorType::Value::GOOVIS);
        } else if (input_type == "PEDALS_ISI_HEAD_SENSOR" || input_type == "PEDALS_DVRK_HEAD_SENSOR") {
            std::string io_file = value["IO_head_sensor"].get("IO_file", "").asString();
            if (io_file == "") { return nullptr; }

            std::string prefix = "sawRobotIO1394-";
            std::string suffix = "-dv-head-sensor";
            size_t start_idx = io_file.find(prefix) + prefix.size();
            size_t end_idx = io_file.find(suffix);

            if (end_idx == std::string::npos || start_idx == std::string::npos || end_idx <= start_idx) {
                return nullptr;
            }

            std::string arm_name = io_file.substr(start_idx, end_idx - start_idx);
            bool is_ISI = input_type == "PEDALS_ISI_HEAD_SENSOR";
            HeadSensorType type = is_ISI ? HeadSensorType::Value::ISI : HeadSensorType::Value::DVRK;
            auto config = std::make_unique<HeadSensorConfig>(type);
            config->source_arm_name = arm_name;
            return config;
        } else {
            return std::make_unique<HeadSensorConfig>(HeadSensorType::Value::NONE);
        }
    }

    Json::Value toJSON(std::filesystem::path CMN_UNUSED(destination)) const {
        if (type == HeadSensorType::Value::GOOVIS) {
            Json::Value value;
            value["HID_file"] = "hid/goovis-hd.json";
            return value;
        } else if (type == HeadSensorType::Value::ISI || type == HeadSensorType::Value::DVRK) {
            auto source = getSource();
            if (!source.has_value()) { return Json::Value(); }

            Json::Value value;
            Json::Value io;
            io["IO"] = source->io_name.value_or("");
            io["IO_file"] = "io/sawRobotIO1394-" + source->name + "-dv-head-sensor.json";
            value["IO_head_sensor"] = io;
            return value;
        } else {
            return Json::Value();
        }
    }

    /* Store source arm name, and prefer retrieving IO name from current arm, so that changes to source arm config are reflected here */
    HeadSensorType type;
    std::string source_arm_name;

    void provideSources(ListModelT<ArmConfig> const& arms) {
        this->arms = &arms;
    }

private:
    ListModelT<ArmConfig> const* arms = nullptr;

    std::optional<ArmConfig> getSource() const {
        if (arms == nullptr) { return {}; }

        std::optional<ArmConfig> source = arms->find(
            [this](const ArmConfig& arm){ return arm.name == source_arm_name; }
        );

        return source;
    }
};

class ForceDimensionButtonInputConfig : public QObject {
    Q_OBJECT

public:
    ForceDimensionButtonInputConfig() : QObject() {}

    static std::unique_ptr<ForceDimensionButtonInputConfig> fromJSON(Json::Value json) {
        auto operator_present = json["operator_present"];
        auto clutch = json["clutch"];
        auto camera = json["camera"];

        auto getComponent = [](Json::Value json) -> std::string {
            return json.get("component", "").asString();
        };

        auto getInterface = [](Json::Value json) -> std::string {
            std::string button_interface = json.get("interface", "").asString();
            size_t start_idx = 0;
            size_t end_idx = button_interface.find("/");
            if (end_idx == 0 || end_idx == std::string::npos) {
                return "";
            }

            return button_interface.substr(start_idx, end_idx - start_idx);
        };

        std::string component_name = getComponent(operator_present);
        std::string interface_name = getInterface(operator_present);

        bool components_match = component_name == getComponent(clutch) && component_name == getComponent(camera);
        bool interfaces_match = interface_name == getInterface(clutch) && interface_name == getInterface(camera);

        if (component_name == "" || !components_match) {
            return nullptr;
        }

        if (interface_name == "" || !interfaces_match) {
            return nullptr;
        }

        auto config = std::make_unique<ForceDimensionButtonInputConfig>();
        // Assume interface name used is always the arm name
        config->source_arm_name = interface_name;
        return config;
    }

    Json::Value toJSON(std::filesystem::path destination) const {
        auto source = getSource();
        if (!source.has_value()) { return Json::Value(); }

        auto operator_present = ComponentInterfaceConfig();
        auto clutch = ComponentInterfaceConfig();
        auto camera = ComponentInterfaceConfig();

        if (source->component.has_value()) {
            operator_present = source->component.value();
            operator_present.interface_name += "/center";
            clutch = source->component.value();
            clutch.interface_name += "/top";
            camera = source->component.value();
            camera.interface_name += "/left";
        }

        Json::Value json;
        json["operator_present"] = operator_present.toJSON(destination);
        json["clutch"] = clutch.toJSON(destination);
        json["camera"] = camera.toJSON(destination);
        return json;
    }

    /* Store source arm name, and prefer retrieving IO name from current arm, so that changes to source arm config are reflected here */
    std::string source_arm_name;

    void provideSources(ListModelT<ArmConfig> const& arms) {
        this->arms = &arms;
    }

private:
    ListModelT<ArmConfig> const* arms = nullptr;

    std::optional<ArmConfig> getSource() const {
        if (arms == nullptr) { return {}; }

        std::optional<ArmConfig> source = arms->find(
            [this](const ArmConfig& arm){ return arm.name == source_arm_name; }
        );

        return source;
    }
};

class ConsoleInputConfig : public QObject {
    Q_OBJECT

public:
    ConsoleInputConfig() :
        QObject(),
        type(ConsoleInputType::Value::NONE),
        pedals(std::make_unique<FootPedalsConfig>()),
        head_sensor(std::make_unique<HeadSensorConfig>(HeadSensorType::Value::NONE)),
        force_dimension_buttons(std::make_unique<ForceDimensionButtonInputConfig>())
    {}

    static std::unique_ptr<ConsoleInputConfig> fromJSON(Json::Value value) {
        auto startsWith = [](std::string name, std::string prefix) {
            return name.size() >= prefix.size() && name.substr(0, prefix.size()) == prefix;
        };

        auto config = std::make_unique<ConsoleInputConfig>();

        std::string type_name = value.get("input_type", "").asString();
        if (startsWith(type_name, "PEDALS_")) {
            config->type = ConsoleInputType::Value::FOOT_PEDALS;
        } else if (type_name == "COMPONENTS") {
            config->type = ConsoleInputType::Value::FORCE_DIMENSION_BUTTONS;
        } else {
            config->type = ConsoleInputType::Value::NONE;
        }

        auto pedals = FootPedalsConfig::fromJSON(value["IO_pedals"]);
        if (pedals != nullptr) {
            config->pedals = std::move(pedals);
        }

        auto head_sensor = HeadSensorConfig::fromJSON(value);
        if (head_sensor != nullptr) {
            config->head_sensor = std::move(head_sensor);
        }

        auto force_dimension_buttons = ForceDimensionButtonInputConfig::fromJSON(value);
        if (force_dimension_buttons != nullptr) {
            config->force_dimension_buttons = std::move(force_dimension_buttons);
        }

        return config;
    }

    Json::Value toJSON(std::filesystem::path destination) const {
        Json::Value value;

        if (type == ConsoleInputType::Value::FOOT_PEDALS) {
            value["IO_pedals"] = pedals->toJSON(destination);
            if (head_sensor->type == HeadSensorType::Value::GOOVIS) {
                value["input_type"] = "PEDALS_GOOVIS_HEAD_SENSOR";
            } else if (head_sensor->type == HeadSensorType::Value::DVRK) {
                value["input_type"] = "PEDALS_DVRK_HEAD_SENSOR";
            } else if (head_sensor->type == HeadSensorType::Value::ISI) {
                value["input_type"] = "PEDALS_ISI_HEAD_SENSOR";
            } else {
                value["input_type"] = "PEDALS_ONLY";
            }

            Json::Value sibling = head_sensor->toJSON(destination);
            for (const std::string& k : sibling.getMemberNames()) {
                value[k] = sibling[k];
            }
        } else if (type == ConsoleInputType::Value::FORCE_DIMENSION_BUTTONS) {
            value["input_type"] = "COMPONENTS";

            Json::Value sibling = force_dimension_buttons->toJSON(destination);
            for (const std::string& k : sibling.getMemberNames()) {
                value[k] = sibling[k];
            }
        } else {
            value["input_type"] = "SIMULATED";
        }

        return value;
    }

    void provideSources(ListModelT<ArmConfig> const& arms) {
        pedals->provideSources(arms);
        head_sensor->provideSources(arms);
        force_dimension_buttons->provideSources(arms);
    }

    ConsoleInputType type;

    std::unique_ptr<FootPedalsConfig> pedals;
    std::unique_ptr<HeadSensorConfig> head_sensor;
    std::unique_ptr<ForceDimensionButtonInputConfig> force_dimension_buttons;

signals:
    void updated();
};

class ConsoleConfig : public QObject {
    Q_OBJECT

public:
    ConsoleConfig() : QObject() {
        inputs = std::make_unique<ConsoleInputConfig>();
        psm_teleops = std::make_unique<ListModelT<TeleopConfig>>();
        ecm_teleops = std::make_unique<ListModelT<TeleopConfig>>();

        QObject::connect(inputs.get(), &ConsoleInputConfig::updated, this, &ConsoleConfig::updated);
        QObject::connect(psm_teleops.get(), &ListModel::updated, this, &ConsoleConfig::updated);
        QObject::connect(ecm_teleops.get(), &ListModel::updated, this, &ConsoleConfig::updated);
    }

    static std::unique_ptr<ConsoleConfig> fromJSON(Json::Value json_config) {
        auto config = std::make_unique<ConsoleConfig>();
        Json::Value json_value;

        config->name = json_config["name"].asString();

        config->inputs = ConsoleInputConfig::fromJSON(json_config);
        if (config->inputs == nullptr) {
            return {};
        }
        QObject::connect(config->inputs.get(), &ConsoleInputConfig::updated, config.get(), &ConsoleConfig::updated);

        json_value = json_config["teleop_PSMs"];
        if (!json_value.empty() && json_value.isArray()) {
            config->psm_teleops = ListModelT<TeleopConfig>::fromJSON(json_value);
            if (config->psm_teleops == nullptr) {
                return {};
            }
            QObject::connect(config->psm_teleops.get(), &ListModel::updated, config.get(), &ConsoleConfig::updated);
        }

        json_value = json_config["teleop_ECMs"];
        if (!json_value.empty() && json_value.isArray()) {
            config->ecm_teleops = ListModelT<TeleopConfig>::fromJSON(json_value);
            if (config->ecm_teleops == nullptr) {
                return {};
            }
            QObject::connect(config->ecm_teleops.get(), &ListModel::updated, config.get(), &ConsoleConfig::updated);
        }

        return config;
    }

    Json::Value toJSON(std::filesystem::path destination) const {
        Json::Value value;

        value["name"] = name;
        value["teleop_PSMs"] = psm_teleops->toJSON(destination);
        value["teleop_ECMs"] = ecm_teleops->toJSON(destination);

        // need to merge in console input config
        Json::Value sibling = inputs->toJSON(destination);
        for (const std::string& k : sibling.getMemberNames()) {
            value[k] = sibling[k];
        }

        return value;
    }

    void provideSources(ListModelT<ArmConfig> const& arms) {
        inputs->provideSources(arms);

        for (int idx = 0; idx < psm_teleops->count(); idx++) {
            psm_teleops->ref(idx).provideSources(arms);
        }

        for (int idx = 0; idx < ecm_teleops->count(); idx++) {
            ecm_teleops->ref(idx).provideSources(arms);
        }
    }

    std::string name;

    std::unique_ptr<ConsoleInputConfig> inputs;

    std::unique_ptr<ListModelT<TeleopConfig>> psm_teleops;
    std::unique_ptr<ListModelT<TeleopConfig>> ecm_teleops;

signals:
    void updated();
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

        if (!json_config.isMember("$id")) {
            std::cerr << "No schema id specified, assuming dvrk-system.schema.json" << std::endl;
        }
        if (!json_config.isMember("$version")) {
            std::cerr << "No schema version specified, assuming version 1" << std::endl;
        }

        std::string schema_id = json_config.get("$id", "dvrk-system.schema.json").asString();
        if (schema_id != "dvrk-system.schema.json") {
            std::cerr << "Unexpected schema id \"" << schema_id << "\", aborting. Are you sure file is a dVRK system config?" << std::endl;
            return nullptr;
        }

        std::string schema_version = json_config.get("$version", std::string("1")).asString();
        if (schema_version != "1") {
            std::cerr << "Unsupported schema version " << schema_version << ", aborting" << std::endl;
            return nullptr;
        }

        json_value = json_config["IOs"];
        if (!json_value.empty() && json_value.isArray()) {
            model->io_configs = ListModelT<IOConfig>::fromJSON(json_value);
            if (model->io_configs == nullptr) {
                return nullptr;
            }
            QObject::connect(model->io_configs.get(), &ListModelT<IOConfig>::updated, model.get(), &SystemConfigModel::updated);
        }

        json_value = json_config["arms"];
        if (!json_value.empty() && json_value.isArray()) {
            model->arm_configs = ListModelT<ArmConfig>::fromJSON(json_value);
            if (model->arm_configs == nullptr) {
                return nullptr;
            }
            QObject::connect(model->arm_configs.get(), &ListModelT<ArmConfig>::updated, model.get(), &SystemConfigModel::updated);
        }

        json_value = json_config["consoles"];
        if (!json_value.empty() && json_value.isArray()) {
            model->console_configs = ConsoleList_t::fromJSON(json_value);
            if (model->console_configs == nullptr) {
                return nullptr;
            }

            QObject::connect(model->console_configs.get(), &ConsoleList_t::updated, model.get(), &SystemConfigModel::updated);

            for (int idx = 0; idx < model->console_configs->count(); idx++) {
                model->console_configs->ref(idx).provideSources(*model->arm_configs);
            }
        }

        json_value = json_config["component_manager"]["components"];
        if (!json_value.empty() && json_value.isArray()) {
            for (unsigned int idx = 0; idx < json_value.size(); idx++) {
                auto component = ComponentConfig::fromJSON(json_value[idx]);

                // Attempt to match each custom component to the arms which use it
                for (int arm_idx = 0; arm_idx < model->arm_configs->count(); arm_idx++) {
                    ArmConfig& arm = model->arm_configs->ref(arm_idx);
                    if (arm.component && arm.component->component_name == component.name) {
                        arm.configureComponent(component);
                    }
                }
            }
        }

        return model;
    }

    bool save(std::filesystem::path config_file) const {
        Json::Value config;

        config["$id"] = "dvrk-system.schema.json";
        config["$version"] = "1";

        config["IOs"] = io_configs->toJSON(config_file);
        config["arms"] = arm_configs->toJSON(config_file);
        config["consoles"] = console_configs->toJSON(config_file);

        // Collect any components used by the arms
        Json::Value component_configs;
        for (int idx = 0; idx < arm_configs->count(); idx++) {
            ArmConfig& arm = arm_configs->ref(idx);
            if (arm.component && arm.component->component) {
                component_configs.append(arm.component->component->toJSON(config_file));
            }
        }

        if (component_configs.size() > 0) {
            config["component_manager"]["components"] = component_configs;
        }

        Json::StreamWriterBuilder writer_builder;
        writer_builder["indentation"] = "    "; // four spaces
        auto writer = std::unique_ptr<Json::StreamWriter>(writer_builder.newStreamWriter());

        std::ofstream json_stream;
        json_stream.open(config_file.c_str());
        if (!json_stream.is_open()) {
            return false;
        }

        writer->write(config, &json_stream);
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

#endif // CONFIG_WIZARD_CONFIG_MODEL

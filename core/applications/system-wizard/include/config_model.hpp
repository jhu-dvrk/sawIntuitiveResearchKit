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

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <QtCore>

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
          watchdog_timeout_ms(10.0)
        { }

    std::string name;
    IOPort port;
    IOProtocol protocol;
    double period_ms;
    double watchdog_timeout_ms;
};

class ArmConfig {
public:
    ArmConfig(std::string name, ArmType type, ArmConfigType config_type) : name(name), type(type), config_type(config_type) { }

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

    std::string name;
    TeleopType type;

    std::vector<int> arms;
};

class ConsoleConfig {

};

class SystemConfigModel : public QObject {
    Q_OBJECT

signals:
    void updated();

public:
    ListModelT<IOConfig> io_configs;
    ListModelT<ArmConfig> arm_configs;
    ListModelT<TeleopConfig> teleop_configs;
    ListModelT<ConsoleConfig> console_configs;
};

}

#endif // SYSTEM_WIZARD_CONFIG_MODEL

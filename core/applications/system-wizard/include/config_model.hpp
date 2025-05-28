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

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <QtCore>

namespace system_wizard {

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

    std::string base() const {
        switch (value) {
        case Value::MTM:
        case Value::MTM_DERIVED:
        case Value::MTM_GENERIC:
            return "MTM";
        case Value::PSM:
        case Value::PSM_DERIVED:
        case Value::PSM_GENERIC:
        case Value::PSM_SOCKET:
            return "PSM";
        case Value::ECM:
        case Value::ECM_DERIVED:
        case Value::ECM_GENERIC:
            return "ECM";
        case Value::SUJ_CLASSIC:
        case Value::SUJ_SI:
        case Value::SUJ_FIXED:
            return "SUJ";
        case Value::FOCUS_CONTROLLER:
            return "Focus controller";
        default:
            return "UNKNOWN";
        }
    }

    std::string human_readable() const {
        switch (value) {
        case Value::MTM:
            return "MTM (Master Tool Manipulator)";
        case Value::PSM:
            return "PSM (Patient Side Manipulator)";
        case Value::ECM:
            return "ECM (Endoscopic Camera Manipulator)";
        case Value::SUJ_CLASSIC:
            return "SUJ Classic (Set Up Joints, Classic generation)";
        case Value::SUJ_SI:
            return "SUJ Si (Set Up Joints, Si generation)";
        case Value::SUJ_FIXED:
            return "SUJ Fixed (Set Up Joints, fixed transforms)";
        case Value::MTM_DERIVED:
            return "MTM (derived)";
        case Value::MTM_GENERIC:
            return "MTM (generic)";
        case Value::PSM_DERIVED:
            return "PSM (derived)";
        case Value::PSM_GENERIC:
            return "PSM (generic)";
        case Value::PSM_SOCKET:
            return "PSM (socket)";
        case Value::ECM_DERIVED:
            return "ECM (derived)";
        case Value::ECM_GENERIC:
            return "ECM (generic)";
        case Value::FOCUS_CONTROLLER:
            return "Focus controller";
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

private:
    Value value;
};

class IOConfig {
public:
    double period_ms;
    double watchdog_timeout_ms;
};

class ArmConfig {
public:
    ArmConfig(std::string name, ArmType type) : name(name), type(type) { }

    std::string description() const {
        return name + " (" + type.human_readable() + ")";
    }

    std::string name;
    ArmType type;

    std::optional<std::string> serial_number;
    std::optional<bool> skip_ros_bridge;

    std::optional<std::string> component_name; // if not provided, uses Arm::name
    std::string interface_name;
};

class TeleopConfig {

};

class ConsoleConfig {

};

class SystemConfigModel : public QObject {
    Q_OBJECT

signals:
    void updated();

    void ioAdded(int index);
    void ioUpdated(int index);
    void ioDeleted(int index);

    void armAdded(int index);
    void armUpdated(int index);
    void armDeleted(int index);

    void teleopAdded(int index);
    void teleopUpdated(int index);
    void teleopDeleted(int index);

    void consoleAdded(int index);
    void consoleUpdated(int index);
    void consoleDeleted(int index);

public slots:
    void deleteArm(int index) {
        arms.erase(arms.begin() + index);
        emit armDeleted(index);
        emit updated();
    }

    void deleteIO(int index) {
        ios.erase(ios.begin() + index);
        emit ioDeleted(index);
        emit updated();
    }

public:
    void addArm(ArmConfig arm) {
        arms.push_back(arm);
        emit armAdded(arms.size() - 1);
        emit updated();
    }

    void addIO(IOConfig io) {
        ios.push_back(io);
        emit ioAdded(ios.size() - 1);
        emit updated();
    }

    std::vector<IOConfig> ios;
    std::vector<ArmConfig> arms;
    std::vector<TeleopConfig> teleops;
    std::vector<ConsoleConfig> consoles;
};

}

#endif // SYSTEM_WIZARD_CONFIG_MODEL

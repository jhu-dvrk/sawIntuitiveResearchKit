/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-25

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_ARM_EDITOR
#define SYSTEM_WIZARD_ARM_EDITOR

#include <QtWidgets>

#include "enum_list_model.hpp"

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

    Value idx() const { return value; }

    std::string name() const {
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

    std::string id() const {
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

class ArmEditor : public QFrame {
    Q_OBJECT

public:
    ArmEditor(ArmType type, QWidget* parent = nullptr);

public slots:

signals:

private:
    QComboBox* type_selector;
    EnumListModel<ArmType> type_model;
};

}

#endif // SYSTEM_WIZARD_ARM_EDITOR

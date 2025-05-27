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

#ifndef SYSTEM_WIZARD_IO_EDITOR
#define SYSTEM_WIZARD_IO_EDITOR

#include <QtWidgets>

#include "enum_list_model.hpp"

namespace system_wizard {

class Port {
public:
    enum class Value {
        UDP = 0,
        FIREWIRE = 1,
        UDPFW = 2,
    };

    Port(Value value) : value(value) {}

    static int count() { return 3; }

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

    std::string id() const {
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

class Protocol {
public:
    enum class Value {
        SEQUENTIAL_READ_SEQUENTIAL_WRITE = 0,
        SEQUENTIAL_READ_BROADCAST_WRITE = 1,
        BROADCAST_READ_BROADCAST_WRITE = 2,
    };

    Protocol(Value value) : value(value) {}

    static int count() { return 3; }

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

    std::string id() const {
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

class IOEditor : public QWidget {
    Q_OBJECT

public:
    IOEditor(QWidget* parent = nullptr);

public slots:

signals:

private:
    double period_ms;
    double watchdog_timeout_ms;

    QComboBox* port_selector;
    EnumListModel<Port> port_model;

    QComboBox* protocol_selector;
    EnumListModel<Protocol> protocol_model;
};

}

#endif // SYSTEM_WIZARD_IO_EDITOR

/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2024-01-25

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <hidapi/hidapi.h>

#include <sawIntuitiveResearchKit/mtsHIDHeadSensor.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsHIDHeadSensor, mtsTaskContinuous, mtsTaskContinuousConstructorArg)

class mtsHIDHeadSensorData
{
public:
    hid_device * handle;
};

mtsHIDHeadSensor::mtsHIDHeadSensor(const std::string & componentName):
    mtsTaskContinuous(componentName)
{
    Init();
}

mtsHIDHeadSensor::mtsHIDHeadSensor(const mtsTaskContinuousConstructorArg & arg):
    mtsTaskContinuous(arg)
{
    Init();
}

void mtsHIDHeadSensor::Init(void)
{
    m_data = new mtsHIDHeadSensorData;

    m_interface = AddInterfaceProvided("OperatorPresent");
    if (m_interface) {
        m_interface->AddEventWrite(m_events.operator_present,
                                   "Button", prmEventButton());
    }
}

void mtsHIDHeadSensor::EnumerateDevices(void) const
{
    struct hid_device_info *devs, *cur_dev;
    devs = hid_enumerate(0x0, 0x0);
    cur_dev = devs;
    while (cur_dev) {
        printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
        printf("\n");
        printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
        printf("  Product:      %ls\n", cur_dev->product_string);
        printf("  Release:      %hx\n", cur_dev->release_number);
        printf("  Interface:    %d\n",  cur_dev->interface_number);
        printf("\n");
        cur_dev = cur_dev->next;
    }
    hid_free_enumeration(devs);
}

void mtsHIDHeadSensor::Configure(const std::string & filename)
{
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": failed to parse configuration file \""
                                     << filename << "\"\n"
                                     << jsonReader.getFormattedErrorMessages();
            exit(EXIT_FAILURE);
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                                   << " using file \"" << filename << "\"" << std::endl
                                   << "----> content of configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

        // base component configuration
        mtsComponent::ConfigureJSON(jsonConfig);

        // deserialize configuration
        cmnDataJSON<mtsHIDHeadSensorConfiguration>::DeSerializeText(m_configuration, jsonConfig);

    } catch (std::exception & e) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": parsing file \""
                                 << filename << "\", got error: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
        exit(EXIT_FAILURE);
    }

    // initialize hid
    if (hid_init()) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": hid_init failed!" << std::endl;
    }

    // open the device
    m_data->handle = hid_open(std::stoul(m_configuration.id_vendor, nullptr, 16),
                              std::stoul(m_configuration.id_product, nullptr, 16),
                              NULL);

    // if the device is not found, enumerate existing ones
    if (!m_data->handle) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": failed to find device based on configuration: " << std::endl
                                 << m_configuration << std::endl;
        EnumerateDevices();
        exit(EXIT_FAILURE);
    } else {
        // show some information for debugging
        int res;
        const int MAX_STR = 255;
        wchar_t wstr[MAX_STR];

        // Read the Manufacturer String
        wstr[0] = 0x0000;
        res = hid_get_manufacturer_string(m_data->handle, wstr, MAX_STR);
        if (res < 0)
            printf("Unable to read manufacturer string\n");
        printf("Manufacturer String: %ls\n", wstr);

        // Read the Product String
        wstr[0] = 0x0000;
        res = hid_get_product_string(m_data->handle, wstr, MAX_STR);
        if (res < 0)
            printf("Unable to read product string\n");
        printf("Product String: %ls\n", wstr);

        // Read the Serial Number String
        wstr[0] = 0x0000;
        res = hid_get_serial_number_string(m_data->handle, wstr, MAX_STR);
        if (res < 0)
            printf("Unable to read serial number string\n");
        printf("Serial Number String: (%d) %ls", wstr[0], wstr);
        printf("\n");
    }
}


void mtsHIDHeadSensor::Startup(void)
{
}


void mtsHIDHeadSensor::Run(void)
{
    ProcessQueuedCommands();

    if (!m_data->handle) {
        return;
    }

    unsigned char buf[256];
    int res;

    // request state (cmd 0x81). The first byte is the report number (0x1).
    memset(buf,0x00,sizeof(buf));
    buf[0] = 0x1;
    buf[1] = 0x81;
    res = hid_write(m_data->handle, buf, 17);
    if (res < 0) {
        m_data->handle = nullptr;
        CMN_LOG_CLASS_RUN_ERROR << "Run " << this->GetName()
                                << ": failed to write to device: " << std::endl
                                << m_configuration << std::endl;
        return;
    }

    // read requested state
    res = 0;
    res = hid_read(m_data->handle, buf, sizeof(buf));
    if (res < 0) {
        m_data->handle = nullptr;
        CMN_LOG_CLASS_RUN_ERROR << "Run " << this->GetName()
                                << ": failed to read from device: " << std::endl
                                << m_configuration << std::endl;
        return;
    }

    bool present;

    if (m_configuration.index_data < static_cast<size_t>(res)) {
        present = buf[m_configuration.index_data];
    } else {
        m_data->handle = nullptr;
        CMN_LOG_CLASS_RUN_ERROR << "Run " << this->GetName()
                                << ": invalid data index for device: " << std::endl
                                << m_configuration << std::endl
                                << "Device returned " << res << " data elements" << std::endl;
        return;
    }

    // nothing has changed, don't do anything
    if (m_operating_present == present) {
        return;
    }

    // changed
    m_operating_present = present;
    prmEventButton payload;
    payload.SetValid(true);
    payload.SetTimestamp(StateTable.GetTic());
    if (m_operating_present) {
        payload.SetType(prmEventButton::PRESSED);
        m_events.operator_present(payload);
    } else {
        payload.SetType(prmEventButton::RELEASED);
        m_events.operator_present(payload);
    }
}


void mtsHIDHeadSensor::Cleanup(void)
{
}

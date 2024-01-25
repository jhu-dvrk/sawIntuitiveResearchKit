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

#include <sawIntuitiveResearchKit/mtsGoovisHeadSensor.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsGoovisHeadSensor, mtsTaskContinuous, mtsTaskContinuousConstructorArg)

class mtsGoovisHeadSensorData
{
public:
    hid_device * handle;
};

mtsGoovisHeadSensor::mtsGoovisHeadSensor(const std::string & componentName):
    mtsTaskContinuous(componentName)
{
    Init();
}

mtsGoovisHeadSensor::mtsGoovisHeadSensor(const mtsTaskContinuousConstructorArg & arg):
    mtsTaskContinuous(arg)
{
    Init();
}

void mtsGoovisHeadSensor::Init(void)
{
    m_data = new mtsGoovisHeadSensorData;
}


void mtsGoovisHeadSensor::Configure(const std::string & filename)
{
    std::cerr << "--------------- " << filename << std::endl;
    if (hid_init()) {
        std::cerr << "--------------------- error hid_init " << std::endl;
    }

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

    m_data->handle = hid_open(0x880a, 0x3501, NULL);
    if (!m_data->handle) {
        std::cerr << "--------------------- error hid_open " << std::endl;
    } else {

        int res;
#define MAX_STR 255
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


void mtsGoovisHeadSensor::Startup(void)
{
}


void mtsGoovisHeadSensor::Run(void)
{
    ProcessQueuedCommands();
    std::cerr << ".";

    unsigned char buf[256];
    int res;
    
    // Request state (cmd 0x81). The first byte is the report number (0x1).
    memset(buf,0x00,sizeof(buf));
    buf[0] = 0x1;
    buf[1] = 0x81;
    res = hid_write(m_data->handle, buf, 17);
    if (res < 0)
      printf("Unable to write() (2)\n");

    // Read requested state. hid_read() has been set to be
    // non-blocking by the call to hid_set_nonblocking() above.
    // This loop demonstrates the non-blocking nature of hid_read().
    res = 0;
    res = hid_read(m_data->handle, buf, sizeof(buf));
    if (res < 0)
        printf("Unable to read()\n");

    // Print out the returned buffer.
    // for (int i = 0; i < res; i++) {
    //     printf("%02hhx ", buf[i]);
    // }
    bool operatorPresent = buf[res - 1];
    std::cerr << "op = " << operatorPresent << std::endl;

#if 0
    // nothing has changed, don't do anything
    if (mOperatorPresent == operatorPresent) {
        return;
    }
    
    mOperatorPresent = operatorPresent;

    // changed
    prmEventButton payload;
    payload.SetValid(true);
    payload.SetTimestamp(StateTable.GetTic());
    if (operatorPresent) {
        payload.SetType(prmEventButton::PRESSED);
        MessageEvents.OperatorPresent(payload);
    } else {
        payload.SetType(prmEventButton::RELEASED);
        MessageEvents.OperatorPresent(payload);
    }
#endif
}


void mtsGoovisHeadSensor::Cleanup(void)
{
}

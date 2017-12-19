/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-12-18

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsDaVinciHeadSensor.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDaVinciHeadSensor, mtsTaskFromSignal, mtsTaskConstructorArg)

mtsDaVinciHeadSensor::mtsDaVinciHeadSensor(const std::string & componentName):
    mtsTaskFromSignal(componentName)
{
    Init();
}

mtsDaVinciHeadSensor::mtsDaVinciHeadSensor(const mtsTaskConstructorArg & arg):
    mtsTaskFromSignal(arg)
{
    Init();
}

void mtsDaVinciHeadSensor::Init(void)
{
    // default values
    mSensors.SetAll(0);
    mOperatorPresent = false;

    // Robot IO
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("HeadSensorEnable");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetValue", RobotIO.Enable);
    }
    interfaceRequired = AddInterfaceRequired("HeadSensor1");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciHeadSensor::HeadSensor1EventHandler,
                                                this, "Button");
    }
    mInterface = AddInterfaceProvided("OperatorPresent");
    if (mInterface) {
        mInterface->AddEventWrite(MessageEvents.OperatorPresent,
                                  "Button", prmEventButton());
    }
}


void mtsDaVinciHeadSensor::Configure(const std::string & filename)
{
}


void mtsDaVinciHeadSensor::Startup(void)
{
    RobotIO.Enable(true);
    std::cerr << CMN_LOG_DETAILS << " we should read all current LED status too" << std::endl;
}


void mtsDaVinciHeadSensor::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();
}


void mtsDaVinciHeadSensor::Cleanup(void)
{
    RobotIO.Enable(false);
}


void mtsDaVinciHeadSensor::HeadSensorEventHandler(const size_t sensorNumber,
                                                  const prmEventButton & event)
{
    if (event.Type() == prmEventButton::PRESSED) {
        mSensors.at(sensorNumber) = 1;
    } else {
        mSensors.at(sensorNumber) = 0;
    }
    std::cerr << "Head sensor: " << mSensors.SumOfElements() << std::endl;
}

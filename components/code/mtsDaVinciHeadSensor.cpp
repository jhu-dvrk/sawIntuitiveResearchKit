/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-12-18

  (C) Copyright 2017-2018 Johns Hopkins University (JHU), All Rights Reserved.

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
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("HeadSensorTurnOff");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetValue", RobotIO.TurnOff);
    }
    interfaceRequired = AddInterfaceRequired("HeadSensor1");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciHeadSensor::HeadSensor1EventHandler,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("HeadSensor2");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciHeadSensor::HeadSensor2EventHandler,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("HeadSensor3");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciHeadSensor::HeadSensor3EventHandler,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("HeadSensor4");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciHeadSensor::HeadSensor4EventHandler,
                                                this, "Button");
    }

    mInterface = AddInterfaceProvided("OperatorPresent");
    if (mInterface) {
        mInterface->AddEventWrite(MessageEvents.OperatorPresent,
                                  "Button", prmEventButton());
    }
}


void mtsDaVinciHeadSensor::Configure(const std::string & CMN_UNUSED(filename))
{
}


void mtsDaVinciHeadSensor::Startup(void)
{
    RobotIO.TurnOff(false);
}


void mtsDaVinciHeadSensor::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();
}


void mtsDaVinciHeadSensor::Cleanup(void)
{
    RobotIO.TurnOff(true);
}


void mtsDaVinciHeadSensor::HeadSensorEventHandler(const size_t sensorNumber,
                                                  const prmEventButton & event)
{
    if (event.Type() == prmEventButton::PRESSED) {
        mSensors.at(sensorNumber) = 1;
    } else {
        mSensors.at(sensorNumber) = 0;
    }
    // for now use a simple test, at least two sensors triggered
    const bool operatorPresent = (mSensors.SumOfElements() >= 3);

    // nothing has changed, don't do anything
    if (mOperatorPresent == operatorPresent) {
        return;
    }

    mOperatorPresent = operatorPresent;
}

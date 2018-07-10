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

#ifndef _mtsDaVinciHeadSensor_h
#define _mtsDaVinciHeadSensor_h

#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsDaVinciHeadSensor: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsDaVinciHeadSensor(const std::string & componentName);
    mtsDaVinciHeadSensor(const mtsTaskConstructorArg & arg);
    inline ~mtsDaVinciHeadSensor() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    void Init(void);

    // Required interface
    struct {
        mtsFunctionWrite TurnOff;
    } RobotIO;
    
    // Functions for events
    struct {
        mtsFunctionWrite OperatorPresent;
    } MessageEvents;
    mtsInterfaceProvided * mInterface;

    // callbacks for each event
    inline void HeadSensor1EventHandler(const prmEventButton & event) {
        HeadSensorEventHandler(0, event);
    }
    inline void HeadSensor2EventHandler(const prmEventButton & event) {
        HeadSensorEventHandler(1, event);
    }
    inline void HeadSensor3EventHandler(const prmEventButton & event) {
        HeadSensorEventHandler(2, event);
    }
    inline void HeadSensor4EventHandler(const prmEventButton & event) {
        HeadSensorEventHandler(3, event);
    }

    void HeadSensorEventHandler(const size_t sensorNumber,
                                const prmEventButton & event);

    vctFixedSizeVector<size_t, 4> mSensors;
    bool mOperatorPresent;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDaVinciHeadSensor);

#endif // _mtsDaVinciHeadSensor_h

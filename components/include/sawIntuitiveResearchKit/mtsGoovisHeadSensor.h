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

#ifndef _mtsGoovisHeadSensor_h
#define _mtsGoovisHeadSensor_h

#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// forward declarations for hidapi
class mtsGoovisHeadSensorData;

class CISST_EXPORT mtsGoovisHeadSensor: public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsGoovisHeadSensor(const std::string & componentName);
    mtsGoovisHeadSensor(const mtsTaskContinuousConstructorArg & arg);
    inline ~mtsGoovisHeadSensor() {}

    void Configure(const std::string & filename) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

protected:

    void Init(void);

    // Functions for events
    struct {
        mtsFunctionWrite OperatorPresent;
    } MessageEvents;
    mtsInterfaceProvided * m_interface;

    mtsGoovisHeadSensorData * m_data;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsGoovisHeadSensor);

#endif // _mtsGoovisHeadSensor_h

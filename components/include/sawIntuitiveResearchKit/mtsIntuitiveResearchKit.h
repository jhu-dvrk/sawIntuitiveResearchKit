/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-02-24

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKit_h
#define _mtsIntuitiveResearchKit_h

#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnConstants.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace mtsIntuitiveResearchKit {
    const double PeriodDelay = 0.06 * cmn_ms; // fixed delay
    const double IOPeriod = cmnHzToPeriod(1500.0) - PeriodDelay;
    const double ArmPeriod = cmnHzToPeriod(1500.0) - PeriodDelay;
    const double TeleopPeriod = cmnHzToPeriod(1000.0) - PeriodDelay;
    const double WatchdogTimeout = 30.0 * cmn_ms;

    // DO NOT INCREASE THIS ABOVE 3 SECONDS!!!  Some power supplies
    // (SUJ) will overheat the QLA while trying to turn on power in
    // some specific conditions.  Ask Peter!  See also
    // https://github.com/jhu-cisst/QLA/issues/1
    const double TimeToPower = 3.0 * cmn_s;

    // PSM constants
    const double PSMOutsideCannula = 50.0 * cmn_ms;

    // teleoperation constants
    const double TeleOperationPSMScale = 0.2;
    const double TeleOperationPSMOrientationTolerance = 5.0 * cmnPI_180; // in radians
    const double TeleOperationPSMRollTolerance = 3.0 * cmnPI_180; // in radians
    const double TeleOperationPSMJawRate =  2.0 * cmnPI * cmn_s; // 360 d/s
    const double TeleOperationPSMJawRateBackFromClutch =  0.2 * cmnPI * cmn_s; // 36.0 d/s
};

#endif // _mtsIntuitiveResearchKitArm_h

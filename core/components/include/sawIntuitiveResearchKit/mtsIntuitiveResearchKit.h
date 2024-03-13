/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-02-24

  (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.

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

    const std::string DefaultInstallationDirectory = "/usr/share/sawIntuitiveResearchKit/share";
    const std::string FireWireProtocol = "sequential-read-broadcast-write";

    const std::string crtk_version = "1.1.0";

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

    // joint trajectory ratios
    namespace JointTrajectory {
        const double ratio = 1.0;
        const double ratio_v = 1.0;
        const double ratio_a = 1.0;
    }

    // PSM constants
    namespace PSM {
        // distance in joint space for insertion
        const double EngageDepthCannula = 25.0 * cmn_mm; // approximative depth from cannula tip to RCM
        const double EngageDepthCannulaSnake = 27.0 * cmn_mm; // for snake like instruments (5mm)

        // distance for RCM in cartesian space for first joint at end of instrument's shaft
        const double SafeDistanceFromRCM = 45.0 * cmn_mm;
        // buffer to allow cartesian control to start
        const double SafeDistanceFromRCMBuffer = 2.0 * cmn_mm;

        // range of motion used for 4 last actuators to engage the sterile adapter
        const double AdapterEngageRange = 171.0 * cmnPI_180;

        // maximum range for last 4 actuators when no tool is present
        const double AdapterActuatorLimit = 172.0 * cmnPI_180;

        // disk max torque for engage procedures
        const double DiskMaxTorque = 0.343642;
}

    // MTM constants
    namespace MTMPlatform {
        const double Gain = 0.5; // overall gain applied
        const double PGain = 1.0;
        const double DGain = 0.1;
        const double EffortMax = 0.4;
    }

    // ECM constants
    namespace ECM {
        const double ClassicSDMass = 1.5;
        const double ClassicHDMass = 2.5;
        const double SiHDMass = 0.9;
        const double EmptyMass = 0.05;
    }

    // teleoperation constants
    namespace TeleOperationPSM {
        const double Scale = 0.2;
        const double OrientationTolerance = 5.0 * cmnPI_180; // in radians
        const double GripperThreshold = 5.0 * cmnPI_180; // in radians
        const double RollThreshold = 3.0 * cmnPI_180; // in radians
        const double JawRate =  4.0 * cmnPI * cmn_s; // 720 d/s
        const double JawRateBackFromClutch =  0.2 * cmnPI * cmn_s; // 36.0 d/s
        const double ToleranceBackFromClutch =  2.0 * cmnPI_180; // in radians
    }

    // colors for Si LEDs
    // 200 -> C8
    const uint32_t Red200 = 0xC80000;
    const uint32_t Green200 = 0x00C800;
    const uint32_t Blue200 = 0x0000C8;
    const uint32_t Off = 0x000000;

    // setup logger settings for the dVRK, turn on logs for some
    // classes and create a new log per run with timestamp
    class CISST_EXPORT Logger {
    public:
        Logger(void);
        inline ~Logger() {
            if (m_log_file_stream) {
                Stop();
            }
        }
        void Stop(void);
    private:
        std::ofstream * m_log_file_stream;
    };
};

#endif // _mtsIntuitiveResearchKitArm_h

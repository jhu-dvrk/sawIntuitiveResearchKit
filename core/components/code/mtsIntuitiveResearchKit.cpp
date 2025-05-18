/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2023-10-26

  (C) Copyright 2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnLogger.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>

mtsIntuitiveResearchKit::Logger::Logger(void)
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    // sawRobotIO
    cmnLogger::SetMaskClassMatching("mtsDigital", CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsDallasChip", CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsRobot", CMN_LOG_ALLOW_ALL);
    // sawControllers
    cmnLogger::SetMaskClassMatching("mtsPID", CMN_LOG_ALLOW_ALL);
    // sawIntuitiveResearchKit
    cmnLogger::SetMaskClassMatching("mtsDaVinci", CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsTeleOperation", CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("dvrk_", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    // add log file with date so logs don't get overwritten
    std::string currentDateTime;
    osaGetDateTimeString(currentDateTime);
    m_log_file_stream = new std::ofstream(std::string("cisstLog-" + currentDateTime + ".txt").c_str());
    cmnLogger::AddChannel(*m_log_file_stream);
    cmnLogger::HaltDefaultLog(); // stop log to default cisstLog.txt
}

void mtsIntuitiveResearchKit::Logger::Stop(void)
{
  cmnLogger::Kill();
}

/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/IO_proxy.h>

#include <cisstMultiTask/mtsManagerLocal.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawIntuitiveResearchKit/system.h>

dvrk::IO_proxy::IO_proxy(const std::string & name,
                         dvrk::system * system,
                         dvrk::IO_proxy_configuration * config):
    m_name(name),
    m_system(system),
    m_config(config)
{
}


void dvrk::IO_proxy::post_configure(void)
{
    if (m_config->period > 1.0 * cmn_ms) {
        std::stringstream message;
        message << "post_configure for IO " << m_config->name << std::endl
                << "----------------------------------------------------" << std::endl
                << " Warning:" << std::endl
                << "   The period provided is quite high, i.e. " << m_config->period << std::endl
                << "   seconds.  We strongly recommend you change it to" << std::endl
                << "   a value below 1 ms, i.e. 0.001." << std::endl
                << "----------------------------------------------------";
        std::cerr << "dvrk::post_configure" << message.str() << std::endl;
        CMN_LOG_INIT_WARNING << message.str() << std::endl;
    }

    if (m_config->watchdog_timeout > 300.0 * cmn_ms) {
        m_config->watchdog_timeout = 300.0 * cmn_ms;
        CMN_LOG_INIT_WARNING << "post_configure for IO "
                             << m_config->name
                             << ": watchdog_timeout has to be lower than 300 ms, it has been capped at 300 ms" << std::endl;
    }
    if (m_config->watchdog_timeout <= 0.0) {
        m_config->watchdog_timeout = 0.0;
        std::stringstream message;
        message << "post_configure for IO " << m_config->name << std::endl
                << "----------------------------------------------------" << std::endl
                << " Warning:" << std::endl
                << "   Setting the watchdog timeout to zero disables the" << std::endl
                << "   watchdog.   We strongly recommend to no specify" << std::endl
                << "   io:watchdog_timeout or set it around 30 ms." << std::endl
                << "----------------------------------------------------";
        std::cerr << "dvrk::post_configure" << message.str() << std::endl;
        CMN_LOG_INIT_WARNING << message.str() << std::endl;
    }
}


void dvrk::IO_proxy::create_IO(void)
{
    m_IO = std::make_unique<mtsRobotIO1394>(m_name, m_config->period, m_config->port);
    m_IO->SetProtocol(m_config->protocol);
    m_IO->SetWatchdogPeriod(m_config->watchdog_timeout);
    m_IO->set_calibration_mode(m_calibration_mode);
    for (const auto & config_file : m_config->configuration_files) {
        std::string file = m_system->find_file(config_file);
        if (file == "") {
            CMN_LOG_INIT_ERROR << "IO_proxy::create_IO: can't find IO file " << config_file << std::endl;
            exit(EXIT_FAILURE);
        }
        m_IO->Configure(file);
    }
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    component_manager->AddComponent(m_IO.get());
}


void dvrk::IO_proxy::configure(const std::string & _file)
{
    if (m_IO == nullptr) {
        CMN_LOG_INIT_ERROR << "IO_proxy::configure: create_IO must be called first" << std::endl;
        exit(EXIT_FAILURE);
    }
    m_IO->Configure(_file);
}


void dvrk::IO_proxy::set_calibration_mode(const bool mode)
{
    m_calibration_mode = mode;
    if (m_IO) {
        m_IO->set_calibration_mode(mode);
    }
}

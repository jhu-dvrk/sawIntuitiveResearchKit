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

#include <sawIntuitiveResearchKit/teleop_proxy.h>

#include <cisstMultiTask/mtsManagerLocal.h>

#include <sawIntuitiveResearchKit/system.h>
#include <sawIntuitiveResearchKit/console.h>

dvrk::teleop_proxy::teleop_proxy(const std::string & name,
                                 dvrk::system * system,
                                 dvrk::console * console):
    m_name(name),
    m_system(system),
    m_console(console)
{
}

/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-01-13

  (C) Copyright 2020-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_crtk/mtsROSToCISST.h>

#include <dvrk_psm_from_ros.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(dvrk_psm_from_ros,
                                      dvrk_arm_from_ros,
                                      mtsTaskPeriodicConstructorArg);

dvrk_psm_from_ros::dvrk_psm_from_ros(const std::string & componentName,
                                     cisst_ral::node_ptr_t _node_handle,
                                     const double periodInSeconds)
    : dvrk_arm_from_ros(componentName, _node_handle, periodInSeconds)
{
    InitPSM();
}

dvrk_psm_from_ros::dvrk_psm_from_ros(const mtsTaskPeriodicConstructorArg & arg)
    : dvrk_arm_from_ros(arg)
{
    InitPSM();
}

void dvrk_psm_from_ros::InitPSM(void)
{

    const auto ros_namespace = this->GetName();
    const auto interface_provided = this->GetName();

    typedef std::vector<std::string> Commands;
    populate_interface_provided(interface_provided,
                                ros_namespace,
                                // void commands
                                Commands(),
                                // write commands
                                Commands({"jaw/servo_jp"}),
                                // read commands
                                Commands({"jaw/setpoint_js", "jaw/measured_js"}),
                                // write events
                                Commands());
}

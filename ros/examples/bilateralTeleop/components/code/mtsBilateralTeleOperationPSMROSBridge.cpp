/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "mtsBilateralTeleOperationPSMROSBridge.h"

#include <cisst_ros_bridge/mtsROSBridge.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsBilateralTeleOperationPSMROSBridge,
                                      mts_ros_crtk_bridge_provided,
                                      mtsTaskPeriodicConstructorArg);

mtsBilateralTeleOperationPSMROSBridge::mtsBilateralTeleOperationPSMROSBridge(
                                               const std::string & component_name,
                                               cisst_ral::node_ptr_t node_handle,
                                               const double period_in_seconds):
    mts_ros_crtk_bridge_provided(component_name, node_handle, period_in_seconds) {}

mtsBilateralTeleOperationPSMROSBridge::mtsBilateralTeleOperationPSMROSBridge(const mtsTaskPeriodicConstructorArg & arg):
    mts_ros_crtk_bridge_provided(arg) {}

void mtsBilateralTeleOperationPSMROSBridge::Configure(const std::string & teleop_name)
{
    std::string ros_namespace = teleop_name;
    cisst_ral::clean_namespace(ros_namespace);

    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        ("Setting", "bilateral_enabled", ros_namespace + "/bilateral_enabled");

    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        ("Setting", "set_bilateral_enabled",
         ros_namespace + "/set_bilateral_enabled");

    mtsManagerLocal * manager = mtsComponentManager::GetInstance();
    manager->AddComponent(this);
    manager->Connect(teleop_name, "Setting", events_bridge().GetName(), "Setting");
    manager->Connect(teleop_name, "Setting", subscribers_bridge().GetName(), "Setting");
}

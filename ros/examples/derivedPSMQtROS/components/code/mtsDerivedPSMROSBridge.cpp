/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-12-03

  (C) Copyright 2021-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <mtsDerivedPSMROSBridge.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedPSMROSBridge,
                                      mts_ros_crtk_bridge_provided,
                                      mtsTaskPeriodicConstructorArg);

mtsDerivedPSMROSBridge::mtsDerivedPSMROSBridge(const std::string & _component_name,
                                               cisst_ral::node_ptr_t _node_handle,
                                               const double _period_in_seconds):
    mts_ros_crtk_bridge_provided(_component_name, _node_handle, _period_in_seconds)
{
}

mtsDerivedPSMROSBridge::mtsDerivedPSMROSBridge(const mtsTaskPeriodicConstructorArg & arg):
    mts_ros_crtk_bridge_provided(arg)
{
}

void mtsDerivedPSMROSBridge::Configure(const std::string & arm_name)
{
    std::cerr << "--------------------------- >>> " << arm_name << std::endl;
    // namespace is based on name
    std::string _ros_namespace = arm_name;
    cisst_ral::clean_namespace(_ros_namespace);
    _ros_namespace.append("/new_interface/");

    // publishers/subscribers to new interface
    events_bridge().AddPublisherFromEventWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        ("NewInterface", "activated",
         _ros_namespace + "activated");
    events_bridge().AddPublisherFromEventWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        ("NewInterface", "gain",
         _ros_namespace + "gain");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, CISST_RAL_MSG(std_msgs, Bool)>
        ("NewInterface", "activate",
         _ros_namespace + "activate");
    subscribers_bridge().AddSubscriberToCommandWrite<double, CISST_RAL_MSG(std_msgs, Float64)>
        ("NewInterface", "set_gain",
         _ros_namespace + "set_gain");

    mtsManagerLocal * _manager = mtsComponentManager::GetInstance();
    _manager->AddComponent(this);
    _manager->Connect(arm_name, "NewInterface",
                      events_bridge().GetName(), "NewInterface");
    _manager->Connect(arm_name, "NewInterface",
                      subscribers_bridge().GetName(), "NewInterface");

}

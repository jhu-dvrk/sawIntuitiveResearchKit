/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-11-03

  (C) Copyright 2021-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDerivedPSMROSBridge_h
#define _mtsDerivedPSMROSBridge_h

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

class mtsDerivedPSMROSBridge: public mts_ros_crtk_bridge_provided
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);
public:
    mtsDerivedPSMROSBridge(const std::string & _component_name,
                           cisst_ral::node_ptr_t _node_handle,
                           const double _period_in_seconds = 5.0 * cmn_ms);
    mtsDerivedPSMROSBridge(const mtsTaskPeriodicConstructorArg & arg);

    void Configure(const std::string & arm_name) override;
protected:
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDerivedPSMROSBridge);

#endif // _mtsDerivedPSMROSBridge_h

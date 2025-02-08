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

#ifndef _mtsBilateralTeleOperationPSMROSBridge_h
#define _mtsBilateralTeleOperationPSMROSBridge_h

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

class mtsBilateralTeleOperationPSMROSBridge: public mts_ros_crtk_bridge_provided
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);
public:
    mtsBilateralTeleOperationPSMROSBridge(const std::string & component_name,
                           cisst_ral::node_ptr_t node_handle,
                           const double period_in_seconds = 5.0 * cmn_ms);
    mtsBilateralTeleOperationPSMROSBridge(const mtsTaskPeriodicConstructorArg & arg);

    void Configure(const std::string & teleop_name) override;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsBilateralTeleOperationPSMROSBridge);

#endif // _mtsBilateralTeleOperationPSMROSBridge_h

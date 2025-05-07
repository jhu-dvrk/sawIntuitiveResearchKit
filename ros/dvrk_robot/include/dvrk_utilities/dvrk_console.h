/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-05-23

  (C) Copyright 2015-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _dvrk_console_h
#define _dvrk_console_h

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

namespace dvrk {
    class system;
}

class mts_ros_crtk_robot_io_bridge;

namespace dvrk_ros {
    class console: public mts_ros_crtk_bridge_provided
    {
        CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

    public:
        console(const std::string & name,
                cisst_ral::node_ptr_t node_handle,
                const double & publish_rate_in_seconds,
                const double & tf_rate_in_seconds,
                dvrk::system * dVRK_system);

        // methods using CRTK bridge_interface_provided method
        void bridge_interface_provided_arm(const std::string & _component_name,
                                           const std::string & _interface_name,
                                           const double _publish_period_in_seconds,
                                           const double _tf_period_in_seconds);

        void bridge_interface_provided_ECM(const std::string & _component_name,
                                           const std::string & _interface_name,
                                           const double _publish_period_in_seconds,
                                           const double _tf_period_in_seconds);

        void bridge_interface_provided_MTM(const std::string & _component_name,
                                           const std::string & _interface_name,
                                           const double _publish_period_in_seconds,
                                           const double _tf_period_in_seconds);

        void bridge_interface_provided_PSM(const std::string & _component_name,
                                           const std::string & _interface_name,
                                           const double _publish_period_in_seconds,
                                           const double _tf_period_in_seconds);

        // dVRK specific topics
        void add_topics_console(void);
        void add_topics_endoscope_focus(void);
        // IO timing (to be deprecated)
        void add_topics_IO(void);
        // add topics for IO
        void add_topics_IO(const double _publish_period_in_seconds,
                           const bool _read_write);
        // add topics for all PIDs
        void add_topics_PID(const double _publish_period_in_seconds,
                            const bool _read_write);
        // buttons on ECM
        void add_topics_ECM_IO(const std::string & _arm_name,
                               const std::string & _IO_component_name);
        // buttons on PSM
        void add_topics_PSM_IO(const std::string & _arm_name,
                               const std::string & _IO_component_name);
        void add_topics_SUJ_voltages(void);
        void add_topics_teleop_ECM(const std::string & _name);
        void add_topics_teleop_PSM(const std::string & _name);

    protected:
        mtsROSBridge * m_pub_bridge;
        double m_publish_rate, m_tf_rate;
        dvrk::system * m_system;
    };
}

typedef dvrk_ros::console dvrk_ros_console;
CMN_DECLARE_SERVICES_INSTANTIATION(dvrk_ros_console);

#endif // _dvrk_console_h

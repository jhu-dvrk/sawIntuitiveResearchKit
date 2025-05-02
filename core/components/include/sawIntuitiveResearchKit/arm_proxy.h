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


#ifndef _dvrk_arm_proxy_h
#define _dvrk_arm_proxy_h

#include <cisstMultiTask/mtsFunctionVoid.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawIntuitiveResearchKit/arm_proxy_configuration.h>
#include <sawIntuitiveResearchKit/generation.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class mtsIntuitiveResearchKitConsole;
class mtsIntuitiveResearchKitArm;

namespace dvrk {

    class CISST_EXPORT arm_proxy {
     public:

        friend class ::mtsIntuitiveResearchKitConsole;

        arm_proxy(const std::string & name,
                  mtsIntuitiveResearchKitConsole * console,
                  dvrk::arm_proxy_configuration * config);

        NOT_COPYABLE(arm_proxy);
        NOT_MOVEABLE(arm_proxy);

        /*! Load and validate configuration from json */
        void post_configure(void);

        /*! Create and configure the robot arm. */
        void create_arm(void);

        /*! Create a new PID component and connect it to the proper RobotIO
          interface.  If the period in seconds is zero, the PID will be tied to
          IO using the ExecIn/ExecOut interfaces. */
        void create_PID(void);
        void configure_IO(void);

        /*! Check if mBaseFrame has a valid name and if it does
          set_base_frame on the arm. */
        void set_base_frame_if_needed(void);

        /*! Connect all interfaces specific to this arm. */
        bool connect(void);

        /*! Accessors */
        dvrk::generation generation(void) const;

        std::string m_name;
        mtsIntuitiveResearchKitConsole * m_console = nullptr;
        dvrk::arm_proxy_configuration * m_config = nullptr;

        std::shared_ptr<mtsIntuitiveResearchKitArm> m_arm = nullptr;

        std::string m_serial;
        bool m_calibration_mode = false;

        // low level
        std::string m_IO_component_name;
        std::string m_IO_interface_name;
        std::string m_IO_configuration_file;
        // for MTMs only
        std::string m_IO_gripper_interface_name;
        std::string m_IO_gripper_configuration_file;
        // PID
        std::string m_PID_component_name;
        std::string m_PID_configuration_file; // actual file
        // arm
        std::string m_arm_configuration_file;
        std::string m_arm_component_name; // for generic or derived
        std::string m_arm_interface_name; // for generic or derived

        double m_arm_period = mtsIntuitiveResearchKit::ArmPeriod;
        // add ROS bridge
        bool m_skip_ROS_bridge;

    protected:
        // base frame
        // (name and frame) OR (component and interface)
        prmPositionCartesianSet m_base_frame;
        std::string m_base_frame_component_name;
        std::string m_base_frame_interface_name;

        mtsFunctionWrite state_command;
        mtsFunctionVoid hold;

        // interfaces used to communicate with components created for a given arm
        mtsInterfaceRequired * m_IO_interface_required = nullptr;
        mtsInterfaceRequired * m_IO_dallas_interface_required = nullptr;
        mtsInterfaceRequired * m_PID_interface_required = nullptr;
        mtsInterfaceRequired * m_arm_interface_required = nullptr;

        // this is used only by PSMs and ECM
        mtsInterfaceRequired * SUJInterfaceRequiredFromIO = nullptr;
        mtsInterfaceRequired * SUJInterfaceRequiredFromIO2 = nullptr; // for Si second clutch button
        mtsInterfaceRequired * SUJInterfaceRequiredToSUJ = nullptr;
        mtsFunctionWrite SUJClutch;
        bool m_SUJ_clutched = false;

        void SUJClutchEventHandlerFromIO(const prmEventButton & button) {
            if (button.Type() == prmEventButton::PRESSED) {
                m_SUJ_clutched = true;
                SUJClutch(true);
            } else {
                m_SUJ_clutched = false;
                SUJClutch(false);
            }
        }

        void CurrentStateEventHandler(const prmOperatingState & currentState);
    };
}

#endif // _dvrk_arm_proxy_h

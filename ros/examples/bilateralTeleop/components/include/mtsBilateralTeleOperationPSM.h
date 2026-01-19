/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Brendan Burkhart
  Created on: 2025-01-23

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsBilateralTeleOperationPSM_h
#define _mtsBilateralTeleOperationPSM_h

#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmStateCartesian.h>

// Always include last
#include <sawIntuitiveResearchKitBilateralTeleopExport.h>

class CISST_EXPORT mtsBilateralTeleOperationPSM: public mtsTeleOperationPSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsBilateralTeleOperationPSM(const std::string & componentName, const double periodInSeconds);
    mtsBilateralTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsBilateralTeleOperationPSM() {}

    void Configure(const Json::Value & jsonConfig) override;

    void set_bilateral_enabled(const bool & enabled);

protected:
    class ForceSource {
    public:
        void Configure(mtsBilateralTeleOperationPSM* teleop, const Json::Value & jsonConfig);

        mtsFunctionRead measured_cf;
        prmForceCartesianGet m_measured_cf;

        mtsBilateralTeleOperationPSM* teleop;
        std::string component_name;
        std::string provided_interface_name;
        std::string function_name;
    };

    class Arm {
    public:
        Arm(mtsBilateralTeleOperationPSM* teleop) : teleop(teleop) {}
        virtual ~Arm() {};

        virtual void populateInterface(mtsInterfaceRequired* interface);
        virtual void add_force_source(std::unique_ptr<ForceSource> source) { force_source = std::move(source); }

        virtual prmStateCartesian computeGoal(Arm* target, double scale);

        virtual vctFrm4x4& ClutchOrigin() = 0;

        virtual prmStateCartesian state();
        virtual void servo(prmStateCartesian goal);

    protected:
        mtsBilateralTeleOperationPSM* teleop;
        std::unique_ptr<ForceSource> force_source;

        mtsFunctionWrite servo_cpvf;
        mtsFunctionRead measured_cs;
    };

    class ArmMTM : public Arm {
    public:
        ArmMTM(mtsBilateralTeleOperationPSM* teleop) : Arm(teleop) {}
        ~ArmMTM() {}

        vctFrm4x4& ClutchOrigin() override;
        
        prmStateCartesian state() override;
        void servo(prmStateCartesian goal) override;

        mtsFunctionWrite servo_cp;
        prmPositionCartesianSet m_servo_cp;
    };

    class ArmPSM : public Arm {
    public:
        ArmPSM(mtsBilateralTeleOperationPSM* teleop) : Arm(teleop) {}
        ~ArmPSM() {}

        vctFrm4x4& ClutchOrigin() override;
        
        prmStateCartesian state() override;
        void servo(prmStateCartesian goal) override;

        mtsFunctionRead  measured_cp;
        mtsFunctionRead  measured_cv;
        prmPositionCartesianGet m_measured_cp;
        prmVelocityCartesianGet m_measured_cv;
    };

    ArmMTM mArmMTM;
    ArmPSM mArmPSM;

    bool m_bilateral_enabled;
    mtsFunctionWrite bilateral_enabled_event;

    double m_mtm_torque_gain = 0.2;

    void Init() override;

    void RunCartesianTeleop() override;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsBilateralTeleOperationPSM);

#endif // _mtsBilateralTeleOperationPSM_h

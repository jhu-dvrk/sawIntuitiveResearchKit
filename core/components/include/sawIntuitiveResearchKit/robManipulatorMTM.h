/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-11-11

  (C) Copyright 2019-2020 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#ifndef _robManipulatorMTM_h
#define _robManipulatorMTM_h

#include <cisstRobot/robManipulator.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class robManipulatorMTM: public robManipulator
{

public:
    robManipulatorMTM(const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    robManipulatorMTM(const std::string& robotfilename,
                      const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    robManipulatorMTM(const std::vector<robKinematics *> linkParms,
                      const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    ~robManipulatorMTM() {}

#if CISST_HAS_JSON
    robManipulator::Errno LoadRobot(const Json::Value & config) override;
#endif

    robManipulator::Errno
    InverseKinematics(vctDynamicVector<double> & q,
                      const vctFrame4x4<double> & Rts,
                      double tolerance = 1e-12,
                      size_t Niterations = 1000,
                      double LAMBDA = 0.001);

private:
    // length from shoulder to elbow
    double upper_arm_length;
    // angle formed by gimbal-elbow-forearm
    double elbow_to_gimbal_angle;
    // length from elbow to gimbal center of rotation
    double elbow_to_gimbal_length;

    // rotation to align zero-position of frame 7 with frame 4
    static const vctRot3 rotation_78;

    // allowed (absolute) tolerance when enforcing joint limits
    static constexpr double joint_limit_tolerance = 1e-5;

    static double SolveTriangleInteriorAngle(double side_a, double side_b, double side_c);
    static double ClosestAngleToJointRange(double angle, double modulus, double min, double max);

    double ChoosePlatformYaw(const vctRot3& rotation_47) const;

    vct3 ShoulderElbowIK(const vct3& position_07) const;
    vct3 WristGimbalIK(const vctRot3& rotation_47) const;
};

#endif // _robManipulatorMTM_h

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
#include <cisstNumerical/nmrLSEISolver.h>

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

    robManipulator::Errno
    InverseKinematics(vctDynamicVector<double> & q,
                      const vctFrame4x4<double> & Rts,
                      double tolerance = 1e-12,
                      size_t Niterations = 1000,
                      double LAMBDA = 0.001);

private:
    // values (in meters) taken from jhu-dVRK/kinematic/mtm(rl).json:
    // length from shoulder to elbow
    static constexpr double upper_arm_length_m = 0.2794;
    // legnth from elbow to end of forearm
    static constexpr double forearm_length_m = 0.3645;
    // length from end of forearm to wrist/gimbal's center of rotation
    static constexpr double forearm_to_gimbal_m = 0.1506;
    // Rotation to align zero-position of frame 7 with frame 4
    static const vctRot3 rotation_78;

    // allowed (absolute) tolerance when enforcing joint limits
    static constexpr double joint_limit_tolerance = 1e-5;

    double SolveTriangleInteriorAngle(double side_a, double side_b, double side_c) const;

    double ChoosePlatformYaw(const vctRot3& rotation_47) const;

    vct3 ShoulderElbowIK(const vct3& position_07) const;
    vct3 WristGimbalIK(const vctRot3& rotation_47) const;
};

#endif // _robManipulatorMTM_h

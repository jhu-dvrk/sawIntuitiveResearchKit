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

#include <sawIntuitiveResearchKit/robManipulatorMTM.h>
#include <math.h>

robManipulatorMTM::robManipulatorMTM(const std::vector<robKinematics *> linkParms,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(linkParms, Rtw0)
{
}

robManipulatorMTM::robManipulatorMTM(const std::string &robotfilename,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(robotfilename, Rtw0)
{
}

robManipulatorMTM::robManipulatorMTM(const vctFrame4x4<double> &Rtw0)
    : robManipulator(Rtw0)
{
}

robManipulator::Errno
robManipulatorMTM::InverseKinematics(vctDynamicVector<double> & q,
                                     const vctFrame4x4<double> & Rts,
                                     double CMN_UNUSED(tolerance),
                                     size_t CMN_UNUSED(Niterations),
                                     double CMN_UNUSED(LAMBDA))
{
    if (q.size() != links.size()) {
        std::stringstream ss;
        ss << "robManipulatorMTM::InverseKinematics: expected " << links.size()
           << " joints values but received " << q.size();
        mLastError = ss.str();
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() == 0) {
        mLastError = "robManipulatorMTM::InverseKinematics: the manipulator has no links";
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    // if we encounter a joint limit, keep computing a solution but at
    // the end return failure
    bool hasReachedJointLimit = false;

    // take Rtw0 into account
    vctFrm4x4 Rt07;
    Rtw0.ApplyInverseTo(Rts, Rt07);

    q[0] = atan2l(Rt07.Translation().X(),
                  -Rt07.Translation().Y());

    // arm is provided in ISI DH
    const double l1 = 0.2794;
    const double l1_sqr = l1 * l1;

    // create a triangle "above" forarm to find position
    const double forarmBase = 0.3645; // from ISI documentation
    const double forarmHeight = 0.1506; // for ISI documentation
    const double l2_sqr = forarmBase * forarmBase + forarmHeight * forarmHeight;
    const double l2 = sqrt(l2_sqr) ;
    const double angleOffset = asinl(forarmHeight / l2);

    // project in plane formed by links 2 & 3 to find q2 and q3 (joint[1] and joint[2])
    const double x = -Rt07.Translation().Z();
    const double y = sqrt(Rt07.Translation().X() * Rt07.Translation().X()
                          + Rt07.Translation().Y() * Rt07.Translation().Y());

    // 2 dof IK in plane
    const double d_sqr = x * x + y * y;
    const double d = sqrt(d_sqr);
    const double a1 = atan2l(y, x);
    const double a2 = acosl((l1_sqr - l2_sqr + d_sqr) / (2.0 * l1 * d));
    const double q1 = a1 - a2;
    const double q2 = -acosl((l1_sqr + l2_sqr - d_sqr) / (2.0 * l1 * l2));

    q[1] = q1;
    q[2] = q2 - angleOffset + cmnPI_2;

    // check joint limits for first 3 joints
    for (size_t joint = 0; joint < 3; joint++) {
        if (ClampJointValueAndUpdateError(joint, q[joint], 1e-5)) {
            hasReachedJointLimit = true;
        }
    }

    // optimized placement of platform
    // compute projection of roll axis on platform plane
    q[3] = FindOptimalPlatformAngle(q, Rt07);

    // compute orientation of platform
    const vctFrm4x4 Rt04 = this->ForwardKinematics(q, 4);
    vctFrm4x4 Rt47;
    Rt04.ApplyInverseTo(Rt07, Rt47);
    vctEulerZXZRotation3 closed57(Rt47.Rotation());

    // applying DH offsets
    q[4] = closed57.alpha() + cmnPI_2;
    q[5] = -closed57.beta() + cmnPI_2;
    q[6] = closed57.gamma() + cmnPI;

    if (hasReachedJointLimit) {
        return robManipulator::EFAILURE;
    }

    return robManipulator::ESUCCESS;
}

int method = 2;
bool print_fk = true;

// METHOD 0 -> RISHI'S METHOD
// METHOD 1 -> ANTON'S METHOD
// METHOD 2 -> ADNAN'S METHOD
double robManipulatorMTM::FindOptimalPlatformAngle(const vctDynamicVector<double> & q,
                                                   const vctFrame4x4<double> & Rt07) const
{
    // RISHI'S METHOD
    if(method == 0){
        const vctFrm4x4 Rt03 = ForwardKinematics(q, 3);
        vctFrm4x4 Rt37;
        Rt03.ApplyInverseTo(Rt07, Rt37);

        // find the angle difference between the gripper and the third joint to calculate auto-correct angle
        double angleDifference = acosl(-Rt37.Element(0, 2) /
                                       sqrt(Rt37.Element(1, 2) * Rt37.Element(1, 2) +
                                            Rt37.Element(0, 2) * Rt37.Element(0, 2)));
        if (Rt37.Element(1, 2) > 0.0) {
            angleDifference = -angleDifference;
        }

        // calculate Angle Option 1 (The correct choice when right-side-up)
        double option1 = angleDifference;

        // calculate Angle Option 2 (The correct choice when upside-down)
        double option2 = option1 - cmnPI;

        // Normalize within joint space
        if (option2 > cmnPI) {
            option2 -= 2.0 * cmnPI;
        } else if (option2 < (-3.0 * cmnPI_2)) {
            option2 += 2.0 * cmnPI;
        }

        // Normalize within joint space
        if ((option2 < -cmnPI)
                && (option2 > -3.0 * cmnPI_2)
                && (q[3] > 0.0)) {
            option2 += 2.0 * cmnPI;
        }

        // Normalize within joint space
        if ((option1 > cmnPI_2)
                && (option1 < cmnPI)
                && (q[3] < 0.0)) {
            option1 -= 2.0 * cmnPI;
        }

        // Choose either Option 1 or Option 2 based on which one is closer to the platform angle
        double solution;
        if (std::abs(q[3] - option2) < std::abs(q[3] - option1)) {
            solution = option2;
        } else {
            solution = option1;
        }

        // average with current position based on projection angle
        const double cosProjectionAngle = std::abs(cos(q[4]));
        double q3 = solution * cosProjectionAngle + q[3] * (1 - cosProjectionAngle);

        // make sure we respect joint limits
        const double q3Max = links[3].GetKinematics()->PositionMax();
        const double q3Min = links[3].GetKinematics()->PositionMin();
        if (q3 > q3Max) {
            q3 = q3Max;
        } else if (q3 < q3Min) {
            q3 = q3Min;
        }

        std::cerr << "\r" << "Joint 3 Value: " << q3 << std::endl;

        return q3;
    }
    // ANTON'S METHOD
    else if(method == 1){
        vctDynamicVector<double> jointGoal(q);
        jointGoal[3] = 0.0;
        const vctFrm4x4 Rt04 = ForwardKinematics(jointGoal, 4);
        vctFrm4x4 Rt47;
        Rt04.ApplyInverseTo(Rt07, Rt47);
        vctEulerZXZRotation3 closed47(Rt47.Rotation());

        // applying DH offsets
        const double q4 = closed47.alpha() + cmnPI_2;
        const double q5 = -closed47.beta() + cmnPI_2;

        double q3;
        // upside-down case
        if ((q4 > -cmnPI_2) && (q4 < cmnPI_2)) {
            q3 = q5;
        } else {
            q3 = -q5;
        }

        // average with current position based on projection angle
        const double cosProjectionAngle = std::abs(cos(q4));
        q3 = q3 * cosProjectionAngle + q[3] * (1 - cosProjectionAngle);

        // make sure we respect joint limits
        const double q3Max = links[3].GetKinematics()->PositionMax();
        const double q3Min = links[3].GetKinematics()->PositionMin();
        if (q3 > q3Max) {
            q3 = q3Max;
        } else if (q3 < q3Min) {
            q3 = q3Min;
        }

        return q3;
    }
    // ADNAN'S METHOD
    else if (method==2){
        //
        // Test all the Forward Kinematic Joints
        //
        if(print_fk){
            vctDynamicVector<double> testJointAngles;
            testJointAngles.resize(7);

            for(int i = 0 ; i < testJointAngles.size() ; i ++){
                testJointAngles[i] = 0.0;
            }

            testJointAngles.resize(8);
            for(int i = 1 ; i < testJointAngles.size() + 1 ; i ++){
                vctFrm4x4 T = ForwardKinematics(testJointAngles, i);
                std::cout << "FK LINK " << i << " IN LINK " << 0 << std::endl;
                std::cout << std::fixed << std::showpoint;
                std::cout.precision(3);
                std::cout << T << "\n --- \n";
            }
            std::cout << "\n";
            print_fk = false;
        }
        //
        //
        //

        // Limits for Wrist Pitch (Joint 5)
        double a_lim_5 = -1.5;
        double b_lim_5 = 1.2;
        double c_lim_5 = 1.8;


        vctEulerYZXRotation3 euler_offset;
        euler_offset.Assign(cmnPI_2, 0, 0);

        vctMatrixRotation3<double, true> Rt8;
        vctEulerToMatrixRotation3(euler_offset, Rt8);

        vctFrm4x4 Rt08, Rt78;
        Rt78.Rotation().Assign(Rt8);
        Rt08 = Rt07 * Rt78;

        vctDynamicVector<double> jointGoal(q);
        jointGoal[3] = 0.0;
        const vctFrm4x4 Rt03 = ForwardKinematics(jointGoal, 3);
        vctFrm4x4 Rt38;
        Rt03.ApplyInverseTo(Rt08, Rt38);

        vctEulerYZXRotation3 closed38(Rt38.Rotation());

        // applying DH offsets
        const double q4 = closed38.alpha();
        const double q5 = closed38.beta();

        double sign;
        double range;
        double centered_val;
        double normalized_val;

        if (a_lim_5 < q4 && q4 <= b_lim_5){
            sign = 1;
        }
        else if (b_lim_5 < q4 & q4 < c_lim_5){
            range = c_lim_5 - b_lim_5;
            normalized_val = (q4 - b_lim_5) / range;
            centered_val = normalized_val - 0.5;
            sign = -centered_val * 2;
//            std::cerr << "MID VAL: " <<  sign << std::endl ;
            sign = 0;
        }
        else{
            sign = -1;
        }



        double Kp_3 = 1.0;
        double Kd_3 = 0.05;
        double e;
        double q3;

        e = q5;

        q3 = Kp_3 * e * sign;

        // make sure we respect joint limits
        const double q3Max = links[3].GetKinematics()->PositionMax();
        const double q3Min = links[3].GetKinematics()->PositionMin();
        if (q3 > q3Max) {
            q3 = q3Max;
        } else if (q3 < q3Min) {
            q3 = q3Min;
        }

        std::cout.precision(3);
        std::cout << std::fixed;
        std::cout << "\r" << "Joint 3: ("  << q3 << "), 4: (" << q4 << "), (5: "  << q5 << ")";

        return q3;
    }
}

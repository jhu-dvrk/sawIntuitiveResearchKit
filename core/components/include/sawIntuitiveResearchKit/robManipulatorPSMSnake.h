/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Simon Leonard, Anton Deguet
  Created on: 2017-03-07

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _robManipulatorPSMSnake_h
#define _robManipulatorPSMSnake_h

#include <sawIntuitiveResearchKit/robManipulatorGeneric.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// Six independent coordinates drive eight links. Each wrist bend is shared
// equally by a pair of joints, entirely within the kinematics boundary.
class CISST_EXPORT robManipulatorPSMSnake: public robManipulator
{
public:
    robManipulatorPSMSnake(const prmConfigurationJoint & fullConfig, const KDL::Chain & fullChain);

    static Eigen::VectorXd FullJoints(const Eigen::VectorXd & active);
    static Eigen::VectorXd ActiveJoints(const Eigen::VectorXd & full);
    int numJoints() const override { return 6; }
    void computeJacobians(const Eigen::VectorXd & joints) override;

protected:
    robManipulatorPSMSnake * raw_clone() const override;
    void internalForwardKinematics(const Eigen::VectorXd & joints,
                                    Eigen::Isometry3d & pose, int segments) override;
    bool internalInverseKinematics(const Eigen::Isometry3d & goal,
                                    Eigen::VectorXd & joints, std::string & error) override;

private:
    const prmConfigurationJoint mFullConfig;
    const KDL::Chain mFullChain;
    robManipulatorGeneric mFullManipulator;
};

#endif // _robManipulatorPSMSnake_h

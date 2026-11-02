/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:
 */

/*
  Author(s):  Anton Deguet
  Created on: 2019-11-11

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#ifndef _robManipulatorECM_h
#define _robManipulatorECM_h

#include <sawIntuitiveResearchKit/robManipulatorGeneric.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

class robManipulatorECM : public robManipulatorGeneric
{
protected:
    robManipulatorECM* raw_clone() const override;

    bool internalInverseKinematics(
        const Eigen::Isometry3d& goal_pose,
        Eigen::VectorXd& joint_positions,
        std::string& error_message
    ) override;

public:
    robManipulatorECM(
        const prmConfigurationJoint& joint_config, const KDL::Chain& chain
    );

private:
    const double insertion_offset;
};

#endif

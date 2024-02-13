/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2023-01-05

  (C) Copyright 2023 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#include <sawIntuitiveResearchKit/prmConfigurationJointFromManipulator.h>

void prmConfigurationJointFromManipulator(const robManipulator & manipulator,
                                          const size_t configuration_js_size,
                                          prmConfigurationJoint & configuration_js)
{
    configuration_js.Name().SetSize(configuration_js_size);
    configuration_js.Type().SetSize(configuration_js_size);
    const size_t manipulator_size = manipulator.links.size();
    std::vector<std::string> names(manipulator_size);
    std::vector<robJoint::Type> types(manipulator_size);
    manipulator.GetJointNames(names);
    manipulator.GetJointTypes(types);
    for (size_t index = 0; index < manipulator_size; ++index) {
        configuration_js.Name().at(index) = names.at(index);
        switch (types.at(index)) {
        case robJoint::HINGE:
            configuration_js.Type().at(index) = PRM_JOINT_REVOLUTE;
            break;
        case robJoint::SLIDER:
            configuration_js.Type().at(index) = PRM_JOINT_PRISMATIC;
            break;
        default:
            configuration_js.Type().at(index) = PRM_JOINT_UNDEFINED;
            break;
        }
    }
    // position limits can be read as is
    configuration_js.PositionMin().SetSize(configuration_js_size, 0.0);
    configuration_js.PositionMax().SetSize(configuration_js_size, 0.0);
    manipulator.GetJointLimits(configuration_js.PositionMin().Ref(manipulator_size),
                               configuration_js.PositionMax().Ref(manipulator_size));
    // efforts
    configuration_js.EffortMin().SetSize(configuration_js_size, 0.0);
    configuration_js.EffortMax().SetSize(configuration_js_size, 0.0);
    manipulator.GetFTMaximums(configuration_js.EffortMax().Ref(manipulator_size));
    configuration_js.EffortMin().NegationOf(configuration_js.EffortMax());
}

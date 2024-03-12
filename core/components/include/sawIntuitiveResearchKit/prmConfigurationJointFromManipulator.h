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

#ifndef _prmConfigurationJointFromManipulator_h
#define _prmConfigurationJointFromManipulator_h

#include <cisstRobot/robManipulator.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

/*! This function extracts some information from cisstRobot
  robManipulator to update a prmConfigurationJoint.  It updates the
  names, types as well as position and effort limits.  The size
  parameter is used to resize the prmConfigurationJoint object.  This
  must be greater or equal to the manipulator's size (e.g. number of
  joints). */
void CISST_EXPORT prmConfigurationJointFromManipulator(const robManipulator & manipulator,
                                                       const size_t configuration_js_size,
                                                       prmConfigurationJoint & result);

#endif // _prmConfigurationJointFromManipulator_h

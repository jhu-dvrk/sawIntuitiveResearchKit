/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-11-18

  (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#ifndef _prmActuatorJointCouplingCheck_h
#define _prmActuatorJointCouplingCheck_h

#include <cisstParameterTypes/prmActuatorJointCoupling.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

/*! This function assumes the position actuator to joint matrix is
  defined and will check that all other matrices are consistant.
  If the other matrices are empty, this method will compute
  them. */
void CISST_EXPORT prmActuatorJointCouplingCheck(const size_t nbJoints,
                                                const size_t nbActuators,
                                                const prmActuatorJointCoupling & input,
                                                prmActuatorJointCoupling & result);

#endif // _prmActuatorJointCouplingCheck_h

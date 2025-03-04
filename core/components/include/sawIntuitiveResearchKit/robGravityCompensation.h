/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Brendan Burkhart
  Created on: 2025-02-03

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#ifndef _robGravityCompensation_h
#define _robGravityCompensation_h

#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstVector/vctTypes.h>

class robGravityCompensation {
public:
    virtual ~robGravityCompensation() {}
    virtual vctVec compute(const prmStateJoint& state, vct3 gravity) = 0;
};

#endif // _robGravityCompensation_h

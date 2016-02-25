/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-    */
/* ex: set filetype=cpp softtabstop=2 shiftwidth=2 tabstop=2 cindent expandtab: */

/*
  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2008-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _robManipulatorLSNorm_h
#define _robManipulatorLSNorm_h

#include <string>
#include <vector>

#include <cisstVector/vctTransformationTypes.h>
#include <cisstRobot/robManipulator.h>

#if CISST_HAS_JSON
#include <json/json.h>
#endif

#include <cisstRobot/robExport.h>

class CISST_EXPORT robManipulatorLSNorm : public robManipulator{

public:

  //! Manipulator generic constructor
  /**
     This constructor initializes a manipulator with the kinematics and dynamics
     contained in a file.
     \param robotfilename The file with the kinematics and dynamics parameters
     \param Rtw0 The offset transformation of the robot base
  */
  robManipulatorLSNorm( const std::string& robotfilename,
                        const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>() );

  robManipulatorLSNorm( const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>() );

  //! Evaluate the inverse kinematics
  /**
     Compute the inverse kinematics. The solution is computed numerically using
     Newton's algorithm.
     \param[input] q An initial guess of the solution
     \param[output] q The inverse kinematics solution
     \param Rts The desired position and orientation of the tool control point
     \param tolerance The error tolerance of the solution
     \param Niteration The maximum number of iterations allowed to find a solution
     \return SUCCESS if a solution was found within the given tolerance and
                     number of iterations. ERROR otherwise.
  */
  virtual
    robManipulator::Errno
    InverseKinematicsLSNorm( vctDynamicVector<double>& q,
                             const vctFrame4x4<double>& Rts,
                             double tolerance=1e-12,
                             size_t Niteration=1000,
                             double LAMBDA=0.001 );

};

#endif

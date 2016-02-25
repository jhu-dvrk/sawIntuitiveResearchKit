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

#include <cisstRobot/robManipulatorLSNorm.h>
#include <cisstNumerical/nmrLSMinNorm.h>

robManipulatorLSNorm::robManipulatorLSNorm( const std::string& linkfile,
                                            const vctFrame4x4<double>& Rtw0 )
  : robManipulator( linkfile, Rtw0 ){}

robManipulatorLSNorm::robManipulatorLSNorm( const vctFrame4x4<double>& Rtw0 )
  : robManipulator( Rtw0 ){}

robManipulatorLSNorm::Errno
robManipulatorLSNorm::InverseKinematicsLSNorm( vctDynamicVector<double>& q,
                                               const vctFrame4x4<double>& Rts,
                                               double tolerance,
                                               size_t Niterations,
                                               double LAMBDA ){

  if( q.size() != links.size() ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
                      << ": Expected " << links.size() << " joints values. "
                      << " Got " << q.size()
                      << std::endl;
    return robManipulator::EFAILURE;
  }

  if( links.size() == 0 ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
                      << ": The manipulator has no links."
                      << std::endl;
    return robManipulator::EFAILURE;
  }

  double ndq=1;               // norm of the iteration error
  size_t i;                   // the iteration counter

  // loop until Niter are executed or the error is bellow the tolerance
  for( i=0; i<Niterations && tolerance<ndq; i++ ){

    // Evaluate the forward kinematics
    vctFrame4x4<double,VCT_ROW_MAJOR> Rt = ForwardKinematics( q );

    // Evaluate the spatial Jacobian (also evaluate the forward kin)
    vctDynamicMatrix<double> J(6, links.size(), VCT_COL_MAJOR);
    JacobianSpatial( q, J );

    // compute the translation error
    vctFixedSizeVector<double,3> dt( Rts[0][3]-Rt[0][3],
                                     Rts[1][3]-Rt[1][3],
                                     Rts[2][3]-Rt[2][3] );

    // compute the orientation error
    // first build the [ n o a ] vectors
    vctFixedSizeVector<double,3> n1( Rt[0][0],  Rt[1][0],  Rt[2][0] );
    vctFixedSizeVector<double,3> o1( Rt[0][1],  Rt[1][1],  Rt[2][1] );
    vctFixedSizeVector<double,3> a1( Rt[0][2],  Rt[1][2],  Rt[2][2] );
    vctFixedSizeVector<double,3> n2( Rts[0][0], Rts[1][0], Rts[2][0] );
    vctFixedSizeVector<double,3> o2( Rts[0][1], Rts[1][1], Rts[2][1] );
    vctFixedSizeVector<double,3> a2( Rts[0][2], Rts[1][2], Rts[2][2] );

    // This is the orientation error
    vctFixedSizeVector<double,3> dr = 0.5*( (n1%n2) + (o1%o2) + (a1%a2) );

    // combine both errors in one R^6 vector
    vctDynamicMatrix<double> e(6, 1, VCT_COL_MAJOR);
    e[0][0] = dt[0]; e[1][0] = dt[1]; e[2][0] = dt[2];
    e[3][0] = dr[0]; e[4][0] = dr[1]; e[5][0] = dr[2];

    //vctDynamicMatrix<double> A( J ), b( e );
    // watch out J and e are modified
    vctDynamicMatrix<double> dqm = nmrLSMinNorm( J, e );
    /*
    vctDynamicMatrix<double> E( b - (A*dqm));
    vctDynamicVector<double> EE(7,
                                E[0][0], E[1][0], E[2][0],
                                E[3][0], E[4][0], E[5][0]);
    std::cout << "E: " << EE.Norm() << std::endl;
    */

    vctDynamicVector<double> dq( 6,
                                 dqm[0][0], dqm[1][0], dqm[2][0],
                                 dqm[3][0], dqm[4][0], dqm[5][0] );
    ndq = dq.Norm();

    // update the solution
    for(size_t j=0; j<links.size(); j++) q[j] += dq[j];

  }


  if( i==Niterations ) return robManipulator::EFAILURE;
  else return robManipulator::ESUCCESS;

}

#include <sawIntuitiveResearchKit/robManipulatorPSMSnake.h>
#include <cisstNumerical/nmrLSEISolver.h>

robManipulatorPSMSnake::robManipulatorPSMSnake(const std::vector<robKinematics *> linkParms,
                                               const vctFrame4x4<double> &Rtw0)
    : robManipulator(linkParms, Rtw0)
{

}

robManipulatorPSMSnake::robManipulatorPSMSnake(const std::string &robotfilename,
                                               const vctFrame4x4<double> &Rtw0)
    : robManipulator(robotfilename, Rtw0)
{

}

robManipulatorPSMSnake::robManipulatorPSMSnake(const vctFrame4x4<double> &Rtw0)
    : robManipulator(Rtw0)
{

}

vctDynamicVector<double>
robManipulatorPSMSnake::ConstrainedRMRC(const vctDynamicVector<double> &q,
                                        const vctFixedSizeVector<double, 6> &vw)
{
    // Ex = f
    vctDynamicMatrix<double> E(2, links.size(), 0.0, VCT_COL_MAJOR);
    vctDynamicMatrix<double> f(2,            1, 0.0, VCT_COL_MAJOR);
    E.at(0, 4) = 1.0;     E.at(0, 7) = -1.0;
    E.at(1, 5) = 1.0;     E.at(1, 6) = -1.0;

    // || Ax - B ||
    vctDynamicMatrix<double> A( 6, links.size(), 0.0, VCT_COL_MAJOR );
    vctDynamicMatrix<double> b( 6, 1, 0.0, VCT_COL_MAJOR );
    JacobianSpatial( q, A );
    for( size_t i=0; i<6; i++ )
    { b[i][0] = vw[i]; }

    vctDynamicMatrix<double> G;
    vctDynamicMatrix<double> h;

    nmrLSEISolver lsei( E, A, G );
    lsei.Solve( E, f, A, b, G, h );

    vctDynamicMatrix<double> X = lsei.GetX();
    vctDynamicVector<double> qd( q.size() );
    for( size_t i=0; i<qd.size(); i++ )
        qd[i] = X[i][0];

    return qd;
}

robManipulator::Errno
robManipulatorPSMSnake::InverseKinematics(vctDynamicVector<double> &q,
                                          const vctFrame4x4<double> &Rts,
                                          double tolerance, size_t Niterations)
{
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
    size_t i=0;
    // loop until Niter are executed or the error is bellow the tolerance
    for( i=0; i<Niterations && tolerance<ndq; i++ ){

        // Evaluate the forward kinematics
        vctFrame4x4<double,VCT_ROW_MAJOR> Rt = ForwardKinematics( q );

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
        vctFixedSizeVector<double,6> e( dt[0], dt[1], dt[2], dr[0], dr[1], dr[2] );

        vctDynamicVector<double> dq = ConstrainedRMRC( q, e );

        // compute the L2 norm of the error
        ndq = dq.Norm();

        // update the solution
        for(size_t j=0; j<links.size(); j++) q[j] += dq[j];
    }

    NormalizeAngles(q);

    if( i==Niterations ) return robManipulator::EFAILURE;
    else return robManipulator::ESUCCESS;
}

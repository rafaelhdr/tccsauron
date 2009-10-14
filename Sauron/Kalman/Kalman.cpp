#include "Kalman.h"

namespace sauron
{

Kalman::Kalman()
{
    // TODO move these values to some const var or to a define
    m_kalman = cvCreateKalman( 3, 3 );

    m_auxiliarMatrix  = cvCreateMat( 3, 1, CV_64FC1 );
    m_auxiliarMatrix2 = cvCreateMat( 3, 1, CV_64FC1 );
}


Kalman::~Kalman()
{
    cvReleaseMat( &m_auxiliarMatrix );
    cvReleaseMat( &m_auxiliarMatrix2 );
    cvReleaseKalman( &m_kalman );
}


void Kalman::setStateTransisitionModel( const Matrix &A )
{
    cvCopy( A, m_kalman->transition_matrix->data.db );
}


void Kalman::setMeasurementModel( const Matrix &H )
{
    cvCopy( H, m_kalman->measurement_matrix->data.db );
}


void Kalman::setProcessNoise( double noise )
{
    cvSetIdentity( m_kalman->process_noise_cov, cvRealScalar( noise ) );
}


void Kalman::setMeasureNoise( double noise )
{
    cvSetIdentity( m_kalman->measurement_noise_cov, cvRealScalar( noise ) );
}


void Kalman::predict( Pose &pose )
{
    *m_auxiliarMatrix = *cvKalmanPredict( m_kalman, NULL );
    pose.X()     = cvmGet( m_auxiliarMatrix, 0, 0 );
    pose.Y()     = cvmGet( m_auxiliarMatrix, 1, 0 );
    pose.Theta() = cvmGet( m_auxiliarMatrix, 2, 0 );
}


void Kalman::correct( const Measure &mes, Pose &pose )
{
    cvmSet( m_auxiliarMatrix, 0, 0, mes.X() );
    cvmSet( m_auxiliarMatrix, 1, 0, mes.Y() );
    cvmSet( m_auxiliarMatrix, 2, 0, mes.Theta() );

    *m_auxiliarMatrix2 = *cvKalmanCorrect( m_kalman, m_auxiliarMatrix );
    pose.X()     = cvmGet( m_auxiliarMatrix2, 0, 0 );
    pose.Y()     = cvmGet( m_auxiliarMatrix2, 1, 0 );
    pose.Theta() = cvmGet( m_auxiliarMatrix2, 2, 0 );
}


}   // namespace sauron
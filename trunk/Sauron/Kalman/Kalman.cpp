#include "Kalman.h"

namespace sauron
{

Kalman::Kalman()
{
    // TODO
}


void Kalman::setStateTransisitionModel( const sauron::Matrix &A )
{
    cvCopy( A, m_kalman->transition_matrix->data.db );
}


void Kalman::setMeasurementModel(const sauron::Matrix &H)
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
    const CvMat *temp = cvKalmanPredict( m_kalman, NULL );
    // TODO
}


void Kalman::correct( Pose &pose /*Measure goes here*/ )
{
    // TODO
    // const CvMat *temp = cvKalmanCorrect( m_kalman, measure );
}


}   // namespace sauron
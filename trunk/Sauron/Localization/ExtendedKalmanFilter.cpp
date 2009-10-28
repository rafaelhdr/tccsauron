#include "ExtendedKalmanFilter.h"

namespace sauron
{

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    // TODO
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
    // TODO
}


void ExtendedKalmanFilter::getPrioriEstimate( const Model &F, const Covariance &Q, 
                                              Pose &estimate,  Covariance &P )
{
    // TODO
}


void ExtendedKalmanFilter::getPosterioriEstimate( const Measure &z, const Model &H, const Covariance &R, 
                                                  Pose &estimate, Covariance &P )
{
    // TODO
}

}   // namespace sauron
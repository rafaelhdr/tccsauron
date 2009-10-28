#ifndef __EXTENDED_KALMAN_FILTER_H__
#define __EXTENDED_KALMAN_FILTER_H__

#include "IKalmanFilter.h"

namespace sauron
{

class ExtendedKalmanFilter : public IKalmanFilter
{
    public:
        ExtendedKalmanFilter();
        ~ExtendedKalmanFilter();

        void getPrioriEstimate( const Model &F, const Covariance &Q, 
                                Pose &estimate,  Covariance &P );

        void getPosterioriEstimate( const Measure &z, const Model &H, const Covariance &R, 
                                    Pose &estimate, Covariance &P );
};

}   //  namespace sauron

#endif  // __EXTENDED_KALMAN_FILTER_H__
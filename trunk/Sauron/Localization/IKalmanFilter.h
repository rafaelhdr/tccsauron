#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "Pose.h"
#include "Matrix.h"

namespace sauron
{

class IKalmanFilter
{
    public:
        // Is there any problem calling PrioriEstimate many more times than PosterioriEstimate
        // If yes, then ILocalizationManager should be responsible to save the last valid priori,
        // so he can provide the value to the entity who needs (Sensor or Dynamic)
        virtual void getPrioriEstimate( const Model &F, const Covariance &Q, 
                                        Pose &estimate,  Covariance &P ) = 0;

        virtual void getPosterioriEstimate( const Measure &z, const Model &H, const Covariance &R, 
                                            Pose &estimate, Covariance &P ) = 0;
};

}   // namespace sauron

#endif // __KALMAN_FILTER_H__
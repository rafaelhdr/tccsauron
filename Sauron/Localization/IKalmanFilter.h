#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "Pose.h"
#include "Matrix.h"

namespace sauron
{

class IKalmanFilter
{
    public:
        // TODO Is there any problem calling PrioriEstimate many more times than PosterioriEstimate
        // If yes, then ILocalizationManager should be responsible to save the last valid priori,
        // so he can provide the value to the entity who needs (Sensor or Dynamic)

        virtual void predict( const Matrix &fValue, const Model &F, const Covariance &Q) = 0;
        virtual void update( const Measure &z, const Matrix &hValue, const Model &H, const Covariance &R) = 0;
		virtual Pose getLatestEstimate() = 0;
		virtual void setLatestEstimate(const Pose& pose) = 0;
		virtual Covariance getLatestCovariance() = 0;
		virtual void setLatestCovariance(const Covariance& covariance) = 0;

};

}   // namespace sauron

#endif // __KALMAN_FILTER_H__
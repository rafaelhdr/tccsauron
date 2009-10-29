#ifndef __SENSOR_MODEL_H__
#define __SENSOR_MODEL_H__

#include "Matrix.h"
#include "Pose.h"

namespace sauron
{

class ISensorModel
{
    public:
		virtual void getEstimate( const Pose &last, 
								  pose_t &hValue, Measure &z, Model &H, Covariance &R ) = 0;

        // Not sure - depends on the final architecture
        virtual bool checkNewEstimateAvailable() = 0;
};

}   // namespace sauron

#endif  // __SENSOR_MODEL_H__
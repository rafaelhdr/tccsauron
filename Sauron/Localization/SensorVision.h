#ifndef __SENSOR_VISION_H__
#define __SENSOR_VISION_H__

#include "ISensorModel.h"

namespace sauron
{

class SensorVision : public ISensorModel
{
    public:
        SensorVision();
        ~SensorVision();

        void getEstimate( const Pose &last, 
                          pose_t &hValue, Measure &z, Model &H, Covariance &R );

        // Not sure - depends on the final architecture
        bool checkNewEstimateAvailable();
};

}   // namespace sauron

#endif  // __SENSOR_SONAR_H__

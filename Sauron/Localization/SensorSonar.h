#ifndef __SENSOR_SONAR_H__
#define __SENSOR_SONAR_H__

#include "ISensorModel.h"

namespace sauron
{

class SensorSonar : public ISensorModel
{
    public:
        SensorSonar();
        ~SensorSonar();

        bool getEstimate( const Pose &last, 
                          Matrix &hValue, Measure &z, Model &H, Covariance &R );

        // Not sure - depends on the final architecture
        bool checkNewEstimateAvailable();
};

}   // namespace sauron

#endif  // __SENSOR_SONAR_H__

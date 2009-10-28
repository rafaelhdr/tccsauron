#ifndef __SENSOR_SONAR_H__
#define __SENSOR_SONAR_H__

#include "Sensor.h"

namespace sauron
{

class SensorSonar : public Sensor
{
    public:
        SensorSonar();
        ~SensorSonar();

        void getEstimate( const Pose &last, Matrix &measure, Matrix &obsModel, Matrix &noise );

        // Not sure - depends on the final architecture
        bool checkNewEstimateAvailable();
};

}   // namespace sauron

#endif  // __SENSOR_SONAR_H__

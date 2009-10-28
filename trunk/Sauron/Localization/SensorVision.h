#ifndef __SENSOR_VISION_H__
#define __SENSOR_VISION_H__

#include "Sensor.h"

namespace sauron
{

class SensorVision : public Sensor
{
    public:
        SensorVision();
        ~SensorVision();

        void getEstimate( const Pose &last, Matrix &measure, Matrix &obsModel, Matrix &noise );

        // Not sure - depends on the final architecture
        bool checkNewEstimateAvailable();
};

}   // namespace sauron

#endif  // __SENSOR_SONAR_H__

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "Matrix.h"
#include "Pose.h"

namespace sauron
{

class Sensor
{
    public:
        virtual void getEstimate( const Pose &last, Matrix &measure, Matrix &obsModel, Matrix &noise ) = 0;

        // Not sure - depends on the final architecture
        virtual bool checkNewEstimateAvailable() = 0;
};

}   // namespace sauron

#endif  // __SENSOR_H__
#ifndef __SENSOR_VISION_H__
#define __SENSOR_VISION_H__

#include "ISensorModel.h"
#include "Vision/VisionModel.h"

namespace sauron
{

class SensorVision : public ISensorModel
{
    public:
        SensorVision();
        ~SensorVision();

        bool getEstimate( const Pose &last, 
                          Matrix &hValue, Measure &z, Model &H, Covariance &R );

        // DEPRECATED - getEstimate now returns a bool that indicates if there is available estimate
        //              who implements getEstimate() has to do this check, to avoid unnecessary processing
        bool checkNewEstimateAvailable();

    private:
        VisionModel  m_visionModel;
        MarkVector   m_associatedMarks;

};

}   // namespace sauron

#endif  // __SENSOR_SONAR_H__

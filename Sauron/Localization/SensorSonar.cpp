#include "SensorSonar.h"

namespace sauron
{

SensorSonar::SensorSonar()
{
    // TODO
}

SensorSonar::~SensorSonar()
{
    // TODO
}

void SensorSonar::getEstimate( const Pose &last, 
                               pose_t &hValue, Measure &z, Model &H, Covariance &R )
{
    // TODO
}


bool SensorSonar::checkNewEstimateAvailable()
{
    // TODO
    return false;
}

}   // namespace sauron
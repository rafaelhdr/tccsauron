#include "SensorVision.h"

namespace sauron
{

SensorVision::SensorVision()
{
    // TODO
}


SensorVision::~SensorVision()
{
    // TODO
}


void SensorVision::getEstimate( const Pose &last, 
                                pose_t &hValue, Measure &z, Model &H, Covariance &R )
{
    // TODO
}


bool SensorVision::checkNewEstimateAvailable()
{
    // TODO
    return false;
}

}   // namespace sauron
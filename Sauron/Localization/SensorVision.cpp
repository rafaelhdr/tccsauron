#include "SensorVision.h"

namespace sauron
{

SensorVision::SensorVision()
{
}


SensorVision::~SensorVision()
{
}


void SensorVision::getEstimate( const Pose &last, 
                                Matrix &measure, Matrix &obsModel, Matrix &noise )
{
    // TODO
}


bool SensorVision::checkNewEstimateAvailable()
{
    // TODO
    return false;
}

}   // namespace sauron
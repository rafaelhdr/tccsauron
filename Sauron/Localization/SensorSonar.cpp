#include "SensorSonar.h"

namespace sauron
{

SensorSonar::SensorSonar()
{
}

SensorSonar::~SensorSonar()
{
}

void SensorSonar::getEstimate( const Pose &last, 
                               Matrix &measure, Matrix &obsModel, Matrix &noise )
{
    // TODO
}


bool SensorSonar::checkNewEstimateAvailable()
{
    // TODO
    return false;
}

}   // namespace sauron
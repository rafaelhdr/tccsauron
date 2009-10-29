#include "SensorVision.h"
#include <cmath>

namespace sauron
{

SensorVision::SensorVision()
{
}


SensorVision::~SensorVision()
{
}


bool SensorVision::getEstimate( const Pose &last, 
                                Matrix &hValue, Measure &z, Model &H, Covariance &R )
{
    m_visionModel.getAssociatedMarks( last, m_associatedMarks );

    if ( m_associatedMarks.size() > 0 )
    {
        uint numMarks = m_associatedMarks.size();

        z.resize( numMarks, 1 );
        H.resize( numMarks, 3 );
        R.resize( numMarks, numMarks );
        hValue.resize( numMarks, 1 );

        double v;
        double z;

        double fu = m_visionModel.getHorizontalFocalDistance();
        double u0 = m_visionModel.getProjectionPlaneHorizontalCenter();

        double sinTheta;
        double cosTheta;
        double diffX;
        double diffY;

        MarkVector::iterator it;
        int index = 0;
        for ( it = m_associatedMarks.begin(); it != m_associatedMarks.end(); ++it, ++index )
        {
            const Point2DFloat markPos = it->getPosition();

            sinTheta = sin( last.Theta() );
            cosTheta = cos( last.Theta() );

            diffX = markPos.X() - last.X();
            diffY = markPos.Y() - last.Y();

            v = cosTheta * diffY - sinTheta * diffX;
            z = cosTheta * diffX + sinTheta * diffY;

            hValue( index, 0 ) = fu * v / z + u0;

            H( index, 0 ) = -fu * (sinTheta * z + cosTheta * v) / (z * z);
            H( index, 1 ) =  fu * (cosTheta * z - sinTheta * v) / (z * z);
            H( index, 2 ) =  fu * ( (v * v) / (z * z) + 1 );
        }
    }

    return false;
}


bool SensorVision::checkNewEstimateAvailable()
{
    // TODO
    return false;
}

}   // namespace sauron
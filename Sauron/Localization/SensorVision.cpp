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
        R = boost::numeric::ublas::identity_matrix<double>( numMarks );
        hValue.resize( numMarks, 1 );

        double aux_v;
        double aux_z;
        double aux_z2;

        double fu    = m_visionModel.getHorizontalFocalDistance();
        double u0    = m_visionModel.getProjectionPlaneHorizontalCenter();
        double sigma = m_visionModel.getSigma();

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

            aux_v  = cosTheta * diffY - sinTheta * diffX;
            aux_z  = cosTheta * diffX + sinTheta * diffY;
            aux_z2 = aux_z * aux_z;

            hValue( index, 0 ) = fu * aux_v / aux_z + u0;

            H( index, 0 ) = -fu * (sinTheta * aux_z + cosTheta * aux_v) / aux_z2;
            H( index, 1 ) =  fu * (cosTheta * aux_z - sinTheta * aux_v) / aux_z2;
            H( index, 2 ) =  fu * ( (aux_v * aux_v) / aux_z2 + 1 );

            R( index, index ) = sigma;
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
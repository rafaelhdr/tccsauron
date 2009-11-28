#ifndef __COORDINATE_CONVERTER_H__
#define __COORDINATE_CONVERTER_H__

#include "CameraParams.h"

namespace sauron
{

class CoordinateConverter
{
    public:
        static double Cam2World_X( double u, double y )
        {
            double div = ( u - CameraParams::getProjectionPlaneHorizontalCenter() );
            if ( div != 0.0 )
                return CameraParams::getHorizontalFocalDistance() * y / div;
            
            return 0.0;
        }

        static double Wordl2Cam_U( double x, double y )
        {
            if ( x != 0.0 )
                return CameraParams::getHorizontalFocalDistance() * y / x + CameraParams::getProjectionPlaneHorizontalCenter();

            return CameraParams::getProjectionPlaneHorizontalCenter();
        }
};  

}   //  namespace sauron

#endif  // __COORDINATE_CONVERTER_H__
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
            return CameraParams::getHorizontalFocalDistance() * y / ( u - CameraParams::getProjectionPlaneHorizontalCenter() );
        }

        static double Wordl2Cam_U( double x, double y )
        {
            return CameraParams::getHorizontalFocalDistance() * y / x + CameraParams::getProjectionPlaneHorizontalCenter();
        }
};  

}   //  namespace sauron

#endif  // __COORDINATE_CONVERTER_H__
#ifndef __CAMERA_PARAMS_H__
#define __CAMERA_PARAMS_H__

namespace sauron
{

class CameraParams
{
    public:
        // TODO Set the values
        static double getHorizontalFocalDistance()          { return 1.000; } // run calibration
        static double getVerticalFocalDistance()            { return 1.000; } // run calibration
        static double getProjectionPlaneHorizontalCenter()  { return 320.0; } // half camera width  
        static double getProjectionPlaneVerticalCenter()    { return 240.0; } // half camera heigth
        static double getAngleOfView()                      { return 2.094; } // radians
};


}   // namespace sauron

#endif  // __CAMERA_PARAMS_H__
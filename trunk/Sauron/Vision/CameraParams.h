#ifndef __CAMERA_PARAMS_H__
#define __CAMERA_PARAMS_H__

namespace sauron
{

class CameraParams
{
    public:
        // TODO Set the values
        static double getHorizontalFocalDistance()          { return 394.8956276848544500; }//274.9938104610965900; } // run calibration
        static double getVerticalFocalDistance()            { return 394.3910703010728300; }//274.2551230486371300; } // run calibration
        static double getProjectionPlaneHorizontalCenter()  { return 161.9690548236380600; } // run calibration 
        static double getProjectionPlaneVerticalCenter()    { return 138.3758441701515100; } // run calibration
        static double getAngleOfView()                      { return 2.094; } // radians
};


}   // namespace sauron

#endif  // __CAMERA_PARAMS_H__
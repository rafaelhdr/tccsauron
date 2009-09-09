#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "Image.h"

namespace sauron
{

class Camera
{
    public:
        Camera();
        ~Camera();

        void getFrame( Image &im );
        
        void setSize( uint width, uint height );
        uint getWidth() const;
        uint getHeight() const;        

    private:
        CvCapture *m_capture;
};

} // namespace sauron 

#endif // __CAMERA_H__
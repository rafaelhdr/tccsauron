#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "Image.h"

#define _USE_DSHOW  1

#if _USE_DSHOW
#include <VideoInput/VideoInput.h>
#endif

namespace sauron
{

class Camera
{
    public:
        Camera();
        ~Camera();

        bool getFrame( Image &im );
        
        void setSize( uint width, uint height );
        uint getWidth() const;
        uint getHeight() const; 

#if _USE_DSHOW
    private:
        videoInput  m_videoInput;
        uint m_width;
        uint m_height;
#else
    private:
        CvCapture   *m_capture;
#endif

    
};

} // namespace sauron 

#endif // __CAMERA_H__
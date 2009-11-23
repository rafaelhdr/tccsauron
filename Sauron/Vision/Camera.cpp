#include "Camera.h"
#include <atlbase.h>

namespace sauron
{

#if _USE_DSHOW

Camera::Camera()
{
    m_videoInput.setVerbose( true );

    int numDevices = m_videoInput.listDevices();
    m_videoInput.setupDevice( 0 );

    m_width  = m_videoInput.getWidth( 0 );
    m_height = m_videoInput.getHeight( 0 ); 

    m_videoInput.setIdealFramerate( 0, 60 );

    //m_videoInput.showSettingsWindow( 0 );

    while ( !m_videoInput.isDeviceSetup( 0 ) );
}

Camera::~Camera()
{
    m_videoInput.stopDevice( 0 );
}


void Camera::getFrame( Image &im )
{
    if ( m_videoInput.isFrameNew( 0 ) )
    {
        if ( im.getWidth() != m_width || im.getHeight() != m_height || im.getFormat() != Pixel::PF_RGB )
            im = Image( m_width, m_height, 8, Pixel::PF_RGB );

        m_videoInput.getPixels( 0, (byte *)((IplImage*)im)->imageData, false, true ); 
    }
}

void Camera::setSize( uint width, uint height )
{
    m_videoInput.stopDevice( 0 );
    m_videoInput.setupDevice( 0, width, height );
    m_width  = m_videoInput.getWidth( 0 );
    m_height = m_videoInput.getHeight( 0 ); 

    while ( !m_videoInput.isDeviceSetup( 0 ) );
}


uint Camera::getWidth() const
{
    return m_width;
}


uint Camera::getHeight() const
{
    return m_height;
}

#else   // _USE_DSHOW

Camera::Camera()
    : m_capture( NULL )
{
    m_capture = cvCaptureFromCAM( CV_CAP_ANY );

    // Ensures that the camera is initialized before exit the constructor
    while ( !cvGrabFrame( m_capture ) );
}

Camera::~Camera(void)
{
    cvReleaseCapture( &m_capture );
}


void Camera::getFrame( Image &im )
{
    IplImage *frame = cvQueryFrame( m_capture );
    im = frame;
}


void Camera::setSize( uint width, uint height )
{
    cvSetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH, width );
    cvSetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT, height );
}


uint Camera::getWidth() const
{
    return (uint)cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
}


uint Camera::getHeight() const
{
    return (uint)cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );
}

#endif  // _USE_DSHOW

}   // namespace sauron

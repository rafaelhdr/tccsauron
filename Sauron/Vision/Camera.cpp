#include "Camera.h"


namespace sauron
{

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

}   // namespace sauron

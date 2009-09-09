#include "Image.h"

namespace sauron
{

Image::Image( uint width, uint height, uint bpp, const Pixel::PixelFormat &pf )
{
    m_image = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, pf );
    cvSetZero( m_image );
}


Image::Image( const std::string &fileName )
    : m_image( NULL )
{
    load( fileName );
}


Image::Image( const Image &other )
{
    m_image = cvCloneImage( other.m_image );
}


Image::~Image()
{
    if ( m_image )
        cvReleaseImage( &m_image );
}

};  // namespace sauron

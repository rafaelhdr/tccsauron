#ifndef __IMAGE_IMPLEMENTATION_H__
#define __IMAGE_IMPLEMENTATION_H__

#include <exception>

#define ImageException( x ) std::exception()

namespace sauron
{

uint Image::getWidth() const
{
    return m_image->width;
}


uint Image::getHeight() const
{
    return m_image->height;
}


uint Image::getBitsPerPixel() const
{
    return m_image->depth * m_image->nChannels;
}


Pixel::PixelFormat Image::getFormat() const
{
    return (Pixel::PixelFormat)m_image->nChannels;
}


void Image::load( const std::string &fileName )
{
    if ( m_image )
        cvReleaseImage( &m_image );
    
    m_image = cvLoadImage( fileName.c_str(), 1 );
    if ( !m_image )
        throw ImageException( "Image file could not be loaded." );
}


void Image::loadGray( const std::string &fileName )
{
    if ( m_image )
        cvReleaseImage( &m_image );
        
    m_image = cvLoadImage( fileName.c_str(), 0 );
    if ( !m_image )
        throw ImageException( "Image file could not be loaded." );
}


void Image::save( const std::string &fileName )
{
    if ( !m_image )
        throw ImageException( "Image could not be saved." );
        
    cvSaveImage( fileName.c_str(), m_image );
}


Image &Image::operator = ( const Image &other )
{
    if ( !other.m_image )
        throw ImageException( "Image to be copied does not exist." );
    
    if ( m_image )
        cvReleaseImage( &m_image );
        
    m_image = cvCloneImage( other.m_image );
    if ( !m_image )
        throw ImageException( "Image could not be copied." );
        
    return (*this);
}


Image &Image::operator = ( const IplImage *other )
{
    if ( !other )
        throw ImageException( "IplImage is null." );

    if ( m_image )
        cvReleaseImage( &m_image );

    m_image = cvCloneImage( other );
    if ( !m_image )
        throw ImageException( "Image could not be copied." );

    return (*this);
}


Image::operator IplImage*()
{
    return m_image;
}


void Image::convertToGray()
{
    if ( getFormat() != Pixel::PF_GRAY )
    {
        Image temp( m_image->width, m_image->height, getBitsPerPixel(), Pixel::PF_GRAY );
        cvCvtColor( m_image, temp.m_image, CV_RGB2GRAY );
        cvReleaseImage( &m_image );
        m_image = cvCloneImage( temp.m_image );
        cvReleaseImage( &temp.m_image );
    }
}


void Image::fill( byte r, byte g, byte b )
{
    cvSet( m_image, CV_RGB( r, g, b ) );
}


Pixel Image::operator () ( uint x, uint y )
{
    if ( x > (uint)m_image->width || y > (uint)m_image->height )
        throw ImageException( "Coordinates out of range." );
        
    return Pixel( (byte *)&m_image->imageData[y * m_image->widthStep + x * m_image->nChannels],
                  (Pixel::PixelFormat)m_image->nChannels, 
                  getBitsPerPixel() );
}


const Pixel Image::operator () ( uint x, uint y ) const
{
    if ( x > (uint)m_image->width || y > (uint)m_image->height )
        throw ImageException( "Coordinates out of range." );
        
    return Pixel( (byte *)&m_image->imageData[y * m_image->widthStep + x * m_image->nChannels],
                  (Pixel::PixelFormat)m_image->nChannels, 
                  getBitsPerPixel() );
}


}; // namespace sauron

#endif /* __IMAGE_IMPLEMENTATION_H__ */

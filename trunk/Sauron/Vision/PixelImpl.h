#ifndef __PIXEL_IMPLEMENTATION_H__
#define __PIXEL_IMPLEMENTATION_H__

#define PixelException( x ) std::exception()

#include <exception>

namespace sauron
{

byte &Pixel::R()
{
    if ( m_pixelFormat <= PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[2];
}

const byte &Pixel::R() const
{
    if ( m_pixelFormat <= PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[2];
}


byte &Pixel::G()
{
    if ( m_pixelFormat <= PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[1];
}


const byte &Pixel::G() const
{
    if ( m_pixelFormat <= PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[1];
}


byte &Pixel::B()
{
    if ( m_pixelFormat <= PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[0];
}


const byte &Pixel::B() const
{
    if ( m_pixelFormat <= PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[0];
}


byte &Pixel::A()
{
    if ( m_pixelFormat <= PF_RGB )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[3];
}


const byte &Pixel::A() const
{
    if ( m_pixelFormat <= PF_RGB )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[3];
}


byte &Pixel::Gray()
{
    if ( m_pixelFormat > PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[0];
}


const byte &Pixel::Gray() const
{
    if ( m_pixelFormat > PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[0];
}


byte &Pixel::operator [] ( uint i )
{
    if ( i > (uint)m_pixelFormat )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[i];
}


const byte &Pixel::operator [] ( uint i ) const
{
    if ( i > (uint)m_pixelFormat )
        throw PixelException( "Invalid format for desired access." );
        
    return m_data[i];
}


Pixel::PixelFormat Pixel::getPixelFormat() const
{
    return m_pixelFormat;
}


uint Pixel::getNumBits() const
{
    return m_bitsPerPixel;
}


void Pixel::set( byte r, byte g, byte b )
{
    if ( m_pixelFormat <= PF_GRAY )
        throw PixelException( "Invalid format for desired access." );
    
    m_data[0] = b;
    m_data[1] = g;
    m_data[2] = r;
}


void Pixel::set( byte r, byte g, byte b, byte a )
{
    if ( m_pixelFormat <= PF_RGB )
        throw PixelException( "Invalid format for desired access." );
    
    m_data[0] = b;
    m_data[1] = g;
    m_data[2] = r;
    m_data[3] = a;
}


}; // namespace sauron

#endif /* __PIXEL_IMPLEMENTATION_H__ */

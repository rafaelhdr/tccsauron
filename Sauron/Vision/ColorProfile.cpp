#include "ColorProfile.h"
#include <algorithm>

#include <iostream>

#define COLOR_PROFILE_DEFAULT_EQUAL_VAR 5
#define COLOR_PROFILE_MISS_RATIO  0.05

namespace sauron
{

ColorProfile::ColorProfile( const Image &im, const DiscretizedLine &line, uint size )
    : m_red( NULL ),
      m_green( NULL ),
      m_blue( NULL )
{
    calculate( im, line, size );
}


ColorProfile::ColorProfile( const ColorProfile &other )
{
    m_size = other.m_size;

    m_red   = new byte [ m_size ];
    m_green = new byte [ m_size ];
    m_blue  = new byte [ m_size ];

    // BUG Does not work
    //std::copy( other.m_red, other.m_red + other.m_size, m_red );
    //std::copy( other.m_green, other.m_green + other.m_size, m_red );
    //std::copy( other.m_blue, other.m_blue + other.m_size, m_red );

    memcpy( m_red, other.m_red, m_size );
    memcpy( m_green, other.m_green, m_size );
    memcpy( m_blue, other.m_blue, m_size );
}


ColorProfile::~ColorProfile()
{
    if ( m_red )
        delete [] m_red;

    if ( m_green )
        delete [] m_green;

    if ( m_blue )
        delete [] m_blue;
}


bool ColorProfile::equals( const ColorProfile &other, byte var ) const
{
    uint missed = 0;
    for ( register uint i = 0; i < m_size; ++i )
    {
        if ( abs( m_red[i] - other.m_red[i] ) > var )
            ++missed;

        if ( abs( m_green[i] - other.m_green[i] ) > var )
            ++missed;

         if ( abs( m_blue[i] - other.m_blue[i] ) > var )
            ++missed;
    }

    if ( missed > (uint)(COLOR_PROFILE_MISS_RATIO * m_size * 3) )
        return false;

    return true;
}


bool ColorProfile::operator == ( const ColorProfile &other ) const
{
    return equals( other, COLOR_PROFILE_DEFAULT_EQUAL_VAR );
}


void ColorProfile::calculate( const Image &im, const DiscretizedLine &line, uint lateral )
{
    m_size = 2 * lateral;

    uint *temp_red   = new uint [ m_size ];
    uint *temp_green = new uint [ m_size ];
    uint *temp_blue  = new uint [ m_size ];
    uint *temp_gray  = new uint [ m_size ];

    std::fill( temp_red, temp_red + m_size, 0 ); 
    std::fill( temp_green, temp_green + m_size, 0 );
    std::fill( temp_blue, temp_blue + m_size, 0 );
    std::fill( temp_gray, temp_gray + m_size, 0 );

    if ( m_red )
        delete [] m_red;

    if ( m_green )
        delete [] m_green;

    if ( m_blue )
        delete [] m_blue;

    m_red   = new byte [ m_size ];
    m_green = new byte [ m_size ];
    m_blue  = new byte [ m_size ];

    uint lineLenght = line.getNumPoints();
    uint imageWidth = im.getWidth();

    uint *meanCount = new uint [ m_size ];
    std::fill( meanCount, meanCount + m_size, 0 );

    uint indexHelper;

    for ( register uint i = 0; i < lineLenght; ++i )
    {
        Point2DInt point = line.getPoint( i );
        for ( register int j = -(int)lateral; j < (int)lateral; ++j )
        {
            indexHelper = j + lateral;

            if ( (int)point.X() + j >= 0 && (uint)(point.X() + j) < imageWidth )
            {
                Pixel pixel = im( point.X() + j, point.Y() );
                temp_red[ indexHelper ]   += pixel.R();
                temp_green[ indexHelper ] += pixel.G();
                temp_blue[ indexHelper ]  += pixel.B();
                temp_gray[ indexHelper ]  += ( temp_red[ indexHelper ] + temp_green[ indexHelper ] + temp_blue[ indexHelper ] ) / 3;
                ++meanCount[ indexHelper ];
            }
        }
    }

    

    for ( register uint i = 0; i < m_size; ++i )
    {
        if ( meanCount[ i ] )
        {
            m_red[ i ]   = (byte)(temp_red[ i ]   / meanCount[ i ]);
            m_green[ i ] = (byte)(temp_green[ i ] / meanCount[ i ]);
            m_blue[ i ]  = (byte)(temp_blue[ i ]  / meanCount[ i ]);
        }
        else
        {
            m_red[ i ]   = 0;
            m_green[ i ] = 0;
            m_blue[ i ]  = 0;
        }
    }

    // TODO  Descobrir o que é maldita equalização do vetor de intensidade do paper do 
    //       Barra e implementar

    delete [] temp_red;
    delete [] temp_green;
    delete [] temp_blue;
    delete [] temp_gray;
    delete [] meanCount;
}

}   // namespace sauron
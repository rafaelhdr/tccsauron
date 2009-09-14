#include "ColorProfile.h"
#include <algorithm>

#include <iostream>

#define COLOR_PROFILE_DEFAULT_EQUAL_VAR 5
#define COLOR_PROFILE_MISS_RATIO  0.05

namespace sauron
{

ColorProfile::ColorProfile( const Image &im, const DiscretizedLine &line, uint size )
{
    calculate( im, line, size );
}


ColorProfile::ColorProfile( const ColorProfile &other )
{
    memcpy( m_left, other.m_left, 3 );
    memcpy( m_right, other.m_right, 3 );
}


ColorProfile::~ColorProfile()
{
}


void ColorProfile::calculate( const Image &im, const DiscretizedLine &line, uint lateral )
{
    int tempLeft[3]  = { 0 };
    int tempRight[3] = { 0 };

    uint lineLenght = line.getNumPoints();
    uint imageWidth = im.getWidth();

    uint countLeft  = 0;
    uint countRight = 0;

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

                if ( j < 0 )
                {
                    tempLeft[0] += pixel.R();
                    tempLeft[1] += pixel.G();
                    tempLeft[2] += pixel.B();

                    ++countLeft;
                }
                else if ( j > 0 )
                {
                    tempRight[0] += pixel.R();
                    tempRight[1] += pixel.G();
                    tempRight[2] += pixel.B();

                    ++countRight;
                }
            }
        }
    }    

    for ( register int i = 0; i < 3; ++i )
    {   
        m_left[i]  = (byte)(tempLeft[i]  / countLeft);
        m_right[i] = (byte)(tempRight[i] / countRight);
    }
}


float ColorProfile::compare( const ColorProfile &other ) const
{
    const float maxVar = 30.0f;

    float ret = 0.0f;
    for ( register uint i = 0; i < 3; ++i )
        ret += 1.0f / 6.0f * ( maxVar - abs( m_left[i]  - other.m_left[i] ) ) +
               1.0f / 6.0f * ( maxVar - abs( m_right[i] - other.m_right[i] ) );

    return ret / maxVar;
}

}   // namespace sauron
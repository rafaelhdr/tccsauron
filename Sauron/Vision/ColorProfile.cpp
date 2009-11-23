#include "ColorProfile.h"

namespace sauron
{

ColorProfile::ColorProfile()
    : m_meanLeft( 0 ),
      m_meanRight( 0 )
{
    memset( m_left, 0, 3 );
    memset( m_right, 0, 3 );
}


ColorProfile::ColorProfile( const Image &im, const DiscretizedLine &line, uint size )
{
    calculate( im, line, size );
}


ColorProfile::ColorProfile( const ColorProfile &other )
{
    memcpy( m_left, other.m_left, 3 );
    memcpy( m_right, other.m_right, 3 );

    m_meanLeft  = other.m_meanLeft;
    m_meanRight = other.m_meanRight;
}


ColorProfile::~ColorProfile()
{
}


void ColorProfile::calculate( const Image &im, const DiscretizedLine &line, uint lateral )
{
    int tempLeft[3]  = { 0 };
    int tempRight[3] = { 0 };

    int tempMeanLeft  = 0;
    int tempMeanRight = 0;

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

                    tempMeanLeft += pixel.R() + pixel.G() + pixel.B();

                    ++countLeft;
                }
                else if ( j > 0 )
                {
                    tempRight[0] += pixel.R();
                    tempRight[1] += pixel.G();
                    tempRight[2] += pixel.B();

                    tempMeanRight += pixel.R() + pixel.G() + pixel.B();

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

    m_meanLeft  = (byte)(tempMeanLeft  / countLeft);
    m_meanRight = (byte)(tempMeanRight / countRight);
}


float ColorProfile::compare( const ColorProfile &other ) const
{
    const float maxVar = 30.0f;
    const float maxMeanVar = 50.0f;

    const float colorProportion = 0.7f;
    const float meanProportion  = 1.0f - colorProportion;

    float ret = 0.0f;
    for ( register uint i = 0; i < 3; ++i )
        ret += colorProportion / 6.0f * ( maxVar - abs( m_left[i]  - other.m_left[i] ) ) +
               colorProportion / 6.0f * ( maxVar - abs( m_right[i] - other.m_right[i] ) ) +
               meanProportion  / 2.0f * ( maxMeanVar - abs( m_meanLeft - other.m_meanLeft ) ) +
               meanProportion  / 2.0f * ( maxMeanVar - abs( m_meanRight - other.m_meanRight ) );

    return ret / ( colorProportion * maxVar + meanProportion * maxMeanVar );
}


void ColorProfile::persist( const ColorProfile &colorProfile, std::ostream &stream ) 
{
    // TODO Insert check for valid output stream
    stream.write( (const char *)colorProfile.m_left, 3 );
    stream.write( (const char *)colorProfile.m_right, 3 );
}


ColorProfile ColorProfile::restore( std::istream &stream )
{
    ColorProfile colorProfile;
    // TODO Insert check for each byte read
    stream.read( (char *)colorProfile.m_left, 3 );
    stream.read( (char *)colorProfile.m_right, 3 );

    return colorProfile;
}

}   // namespace sauron
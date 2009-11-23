#include "Mark.h"

namespace sauron
{


Mark::Mark( const Point2DFloat &pos, const ColorProfile &color, const std::string &description )
    : m_position( pos ),
      m_colorProfile( color ),
      m_description( description )
{
}

float Mark::compare(const sauron::Projection &proj) const
{
    return m_colorProfile.compare( proj.getColorProfile() );
}


const Point2DFloat &Mark::getPosition() const
{
    return m_position;
}


const std::string &Mark::getDescription() const 
{
    return m_description;
}


void Mark::persist( const sauron::Mark &mark, std::ostream &stream )
{
    // TODO Do stream validation
    byte array[ sizeof(Point2DFloat) ] = { 0 };
    byte *pt = array;

    memcpy( pt, &mark.m_position.X(), sizeof(float) ); 
    pt += sizeof( float );
    memcpy( pt, &mark.m_position.Y(), sizeof(float) ); 
    stream.write( (const char *)array, sizeof(Point2DFloat) );

    uint descriptionSize = mark.m_description.size();
    stream.write( (const char *)&descriptionSize, sizeof(descriptionSize) );
    stream.write( mark.m_description.c_str(), descriptionSize );

    ColorProfile::persist( mark.m_colorProfile, stream );
}


Mark Mark::restore( std::istream &stream )
{
    // TODO Do stream data validation
    byte array[ sizeof(Point2DFloat) ] = { 0 };
    byte *pt = array;

    stream.read( (char *)array, sizeof(Point2DFloat) );

    float x;
    float y;
    memcpy( &x, pt, sizeof(float) ); 
    pt += sizeof( float );
    memcpy( &y, pt, sizeof(float) );

    char description[512] = { 0 };
    uint descriptionSize;
    stream.read( (char *)&descriptionSize, sizeof( descriptionSize ) );
    stream.read( description, descriptionSize );

    ColorProfile profile = ColorProfile::restore( stream );

    return Mark( Point2DFloat( x, y ), profile, description );
}


}   // namespace sauron
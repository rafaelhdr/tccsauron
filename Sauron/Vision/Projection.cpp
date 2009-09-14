#include "Projection.h"

#include <iostream>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <cmath>

#define PROJECTION_DEFAULT_COLOR_PROFILE_SIZE   7
#define PROJECTION_DEFAULT_COLOR_PROFILE_VAR    20
#define PROJECTION_DEFAULT_LINE_LENGHT_VAR      5
#define PROJECTION_DEFAULT_LINE_ANGLE_VAR       M_PI / 2.1    

namespace sauron
{

Projection::Projection( const Image &im, DiscretizedLine &line )
    : m_discretizedLine( line ),
      m_colorProfile( im, line, PROJECTION_DEFAULT_COLOR_PROFILE_SIZE )
{
}


Projection::Projection( const Projection &other )
    : m_discretizedLine( other.m_discretizedLine ),
      m_colorProfile( other.m_colorProfile )
{
}


Projection::~Projection()
{
}


const ColorProfile &Projection::getColorProfile() const
{
    return m_colorProfile;
}


const DiscretizedLine &Projection::getDiscretizedLine() const
{
    return m_discretizedLine;
}


float Projection::compare( const Projection &other ) const
{
    return m_colorProfile.compare( other.m_colorProfile );
}


//bool Projection::equals( const Projection &other ) const
//{
//    if ( !m_colorProfile.equals( other.m_colorProfile, PROJECTION_DEFAULT_COLOR_PROFILE_VAR ) )
//        return false;
//
//    //uint numPoints = m_discretizedLine.getNumPoints();
//    //if ( abs( (long)(numPoints - other.m_discretizedLine.getNumPoints()) ) > (int)(numPoints >> 3) )
//    //    return false;
//
//    //if ( abs( m_discretizedLine.getAngle() - other.m_discretizedLine.getAngle() ) > PROJECTION_DEFAULT_LINE_ANGLE_VAR )
//    //    return false;
//
//    return true;
//}


//bool Projection::operator ==( const sauron::Projection &other ) const
//{
//    return equals( other );
//}

}   // namespace sauron
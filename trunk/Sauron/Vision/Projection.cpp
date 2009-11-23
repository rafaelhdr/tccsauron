#include "Projection.h"

#define PROJECTION_DEFAULT_COLOR_PROFILE_SIZE   5

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

}   // namespace sauron
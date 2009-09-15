#include "DiscretizedLine.h"

#include <cmath>
#include <algorithm>

namespace sauron
{

static bool verticalPoint2DIntCmp( Point2DInt a, Point2DInt b )
{
    return a.Y() <= b.Y();
}


static bool horizontalPoint2DIntCmp( Point2DInt a, Point2DInt b )
{
    return a.X() <= b.X();
}


DiscretizedLine::DiscretizedLine()
{
}


DiscretizedLine::DiscretizedLine( const DiscretizedLine &other )
{
    m_points = other.m_points;
}


void DiscretizedLine::addPoint( const Point2DInt &p )
{
    m_points.push_back( p );
}


void DiscretizedLine::sortVertical()
{
    std::sort( m_points.begin(), m_points.end(), verticalPoint2DIntCmp );
}


void DiscretizedLine::sortHorizontal()
{
    std::sort( m_points.begin(), m_points.end(), horizontalPoint2DIntCmp );
}


uint DiscretizedLine::getNumPoints() const
{
    return m_points.size();
}


const Point2DInt &DiscretizedLine::getPoint( uint i ) const
{
    return m_points.at( i );
}


const Point2DInt &DiscretizedLine::operator [] ( uint i ) const
{
    return m_points.at( i );
}


float DiscretizedLine::getTangent() const
{
    Point2DInt max;
    Point2DInt min;

    if ( !getNumPoints() )
        return 0.0f;

    // TODO Force points to be ordered at insertion
    //sortVertical();
    max = getPoint( 0 );
    min = getPoint( getNumPoints() - 1 );

    float deltaX = (float)max.X() - (float)min.X();

    if ( deltaX != 0.0 )
        return ((float)max.Y() - (float)min.Y()) / deltaX;
    else
        return 999999.9f;
}


float DiscretizedLine::getAngle() const
{
    return atan( getTangent() );
}


uint DiscretizedLine::getMeanX() const 
{
    uint mean = 0;
    std::vector< Point2DInt >::const_iterator it;
    for ( it = m_points.begin(); it != m_points.end(); ++it )
        mean += it->X();

    return mean / m_points.size();
}


uint DiscretizedLine::getMeanY() const
{
    uint mean = 0;
    std::vector< Point2DInt >::const_iterator it;
    for ( it = m_points.begin(); it != m_points.end(); ++it )
        mean += it->Y();

    return mean / m_points.size();
}


    
}; // namespace sauron


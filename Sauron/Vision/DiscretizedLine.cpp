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
    name = "";
}


DiscretizedLine::DiscretizedLine( const DiscretizedLine &other )
{
    m_points = other.m_points;
    name = other.name;
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

    // FAZER COM QUE OS PONTOS FIQUEM ORDENADOS NA INSERÇÃO
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


    
}; // namespace sauron


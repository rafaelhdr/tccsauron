#ifndef __MARK_H__
#define __MARK_H__

#include <vector>
#include "Projection.h"
#include "Point2D.h"

namespace sauron
{

class Mark
{
    public:
        Mark();

        const Point2DFloat &getPosition() const;

        float compare( const Projection &proj ) const;

    protected:
        Point2DFloat    m_position;
        ColorProfile    m_colorProfile;
};


typedef std::vector< Mark>  MarkVector;


}   // namespace sauron

#endif // __MARK_H__
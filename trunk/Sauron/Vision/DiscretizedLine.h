#ifndef __DISCRETIZED_LINE_H__
#define __DISCRETIZED_LINE_H__

#include <vector>
#include "BaseTypes.h"
#include "Point2D.h"

#include <string>

namespace sauron
{
    
class DiscretizedLine
{
    public:
        DiscretizedLine();
        DiscretizedLine( const DiscretizedLine &other );
        
        void addPoint( const Point2DInt &p );

        void sortVertical();
        void sortHorizontal();

        uint getNumPoints() const;
        
        const Point2DInt &getPoint( uint i ) const;
        const Point2DInt &operator [] ( uint i ) const;

        float getAngle() const;   // In radians
        float getTangent() const;

        uint getMeanX() const;
        uint getMeanY() const;

    private:
        std::vector< Point2DInt > m_points;        
};

}; // namespace sauron


#endif /* __DISCRETIZED_LINE_H__ */


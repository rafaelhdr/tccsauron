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
        Mark( const Point2DFloat &pos, const ColorProfile &colorProfile, const std::string &description );

        const Point2DFloat &getPosition() const;
        const std::string &getDescription() const;

        float compare( const Projection &proj ) const;

        static void persist( const Mark &mark, std::ostream &stream );
        static Mark restore( std::istream &stream );

    private:
        Mark();

    protected:
        Point2DFloat    m_position;
        ColorProfile    m_colorProfile;
        
        std::string     m_description;
};


typedef std::vector< Mark>  MarkVector;


}   // namespace sauron

#endif // __MARK_H__
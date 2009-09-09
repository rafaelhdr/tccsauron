#ifndef __PROJECTION_H__
#define __PROJECTION_H__

#include <vector>
#include "ColorProfile.h"
#include "DiscretizedLine.h"

namespace sauron
{

class Projection
{
    public:
        Projection( const Image &im, DiscretizedLine &line );
        Projection( const Projection &other );
        ~Projection();

        const ColorProfile &getColorProfile() const;
        const DiscretizedLine &getDiscretizedLine() const;

        bool equals( const Projection &other ) const;
        bool operator == ( const Projection &other ) const;

    private:
        ColorProfile    m_colorProfile;
        DiscretizedLine m_discretizedLine;
};


typedef std::vector< Projection > ProjectionVector;

}   // namespace sauron


#endif // __PROJECTION_H__
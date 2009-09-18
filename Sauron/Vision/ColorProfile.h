#ifndef __COLOR_PROFILE_H__
#define __COLOR_PROFILE_H__

#include "BaseTypes.h"
#include "Image.h"
#include "DiscretizedLine.h"

namespace sauron
{

class ColorProfile
{
    public:
        ColorProfile( const Image &im, const DiscretizedLine &line, uint size );
        ColorProfile( const ColorProfile &other );
        ~ColorProfile();

        float compare( const ColorProfile &other ) const;

        static void persist( const ColorProfile &colorProfile, std::ostream &stream );
        static ColorProfile restore( std::istream &stream );

    private:
        ColorProfile();

        void calculate( const Image &im, const DiscretizedLine &line, uint size );

    private:
        byte   m_left[3];
        byte   m_right[3];
};

}   // namespace sauron

#endif  //  __COLOR_PROFILE_H__
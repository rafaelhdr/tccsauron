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

        bool operator == ( const ColorProfile &other ) const;
        bool equals( const ColorProfile &other, byte var ) const;

    private:
        void calculate( const Image &im, const DiscretizedLine &line, uint size );

    private:
        byte    *m_red;
        byte    *m_green;
        byte    *m_blue;

        uint    m_size;
};

}   // namespace sauron

#endif  //  __COLOR_PROFILE_H__
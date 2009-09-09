#ifndef __PIXEL_H__
#define __PIXEL_H__

#include "BaseTypes.h"

namespace sauron
{

class Image;
    
class Pixel
{
    friend class Image;
    
    public:
        enum PixelFormat
        {
            PF_GRAY = 1,
            PF_RGB  = 3,
            PF_RGBA = 4,
        };
                
        inline byte &R();
        inline const byte &R() const;
        
        inline byte &G();
        inline const byte &G() const;
        
        inline byte &B();
        inline const byte &B() const;
        
        inline byte &A();
        inline const byte &A() const;
        
        inline byte &Gray();
        inline const byte &Gray() const;
        
        inline void set( byte r, byte g, byte b );
        inline void set( byte r, byte g, byte b, byte a );
        
        inline byte &operator [] ( uint i );
        inline const byte &operator [] ( uint i ) const;
        
        inline PixelFormat getPixelFormat() const;
        inline uint getNumBits() const;
        
    private:    
        Pixel( byte *ptPxPos, const PixelFormat &pf, uint bitsPerPixel );

    private:
        uint        m_bitsPerPixel;
        PixelFormat m_pixelFormat; 
        byte        *m_data;
};
    
}; // namespace sauron


#include "PixelImpl.h"


#endif /* __PIXEL_H__ */


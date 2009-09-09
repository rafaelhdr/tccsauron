#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <string>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "BaseTypes.h"
#include "Pixel.h"

namespace sauron
{

class Image
{
    public:
        Image( uint width, uint height, uint bpp, const Pixel::PixelFormat &pf );
        Image( const std::string &fileName );
        Image( const Image &other );
        ~Image();
        
        inline void load( const std::string &fileName );
        inline void loadGray( const std::string &fileName );
        inline void save( const std::string &fileName );

        inline void convertToGray();

        inline void fill( byte r, byte g, byte b );
        
        //*******************
        // Getters
        inline uint getWidth() const;
        inline uint getHeight() const;
        inline uint getBitsPerPixel() const;
        inline Pixel::PixelFormat getFormat() const;
        //*******************
        
        //*******************
        // Operators
        inline Pixel operator () ( uint x, uint y );
        inline const Pixel operator () ( uint x, uint y ) const;
        
        inline Image &operator = ( const Image &other );
        inline Image &operator = ( const IplImage *other );
        inline operator IplImage*();
        //*******************

    private:
        IplImage *m_image;
        
};

}; // namespace sauron


#include "ImageImpl.h"

#endif /* __IMAGE_H__ */


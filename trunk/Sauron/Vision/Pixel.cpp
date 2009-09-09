#include "Pixel.h"

namespace sauron
{

Pixel::Pixel( byte *ptPxPos, const PixelFormat &pf, uint bitsPerPixel )
    : m_bitsPerPixel( bitsPerPixel ),
      m_pixelFormat( pf ),
      m_data( ptPxPos )
{
}

}; // namespace sauron

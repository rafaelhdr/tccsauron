#ifndef __VERTICAL_PROJECTION_DETECTOR_H__
#define __VERTICAL_PROJECTION_DETECTOR_H__

#include "Image.h"
#include "DiscretizedLine.h"
#include "Projection.h"

namespace sauron
{

class VerticalProjectionDetector
{
    public:
        VerticalProjectionDetector();
        ~VerticalProjectionDetector();

        void detect( const Image &colorImage, const Image &grayImage, ProjectionVector &projs ) const;

    private:
        DiscretizedLine recalculateLine( DiscretizedLine &line );


};

} // namespace sauron

#endif // __VERTICAL_PROJECTION_DETECTOR_H__
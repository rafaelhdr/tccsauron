#include <sstream>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "../Vision/Image.h"
#include "../Vision/Projection.h"


void drawProjection( sauron::Image &im, const sauron::Projection &proj, CvFont &font, byte r, byte g, byte b, std::string txt = "" )
{
    sauron::DiscretizedLine line = proj.getDiscretizedLine();
    for ( register sauron::uint k = 0; k < line.getNumPoints(); ++k )
    {   
        sauron::Point2DInt point = line.getPoint( k );
        im( point.X(), point.Y() ).set( r, g, b );
    }
    sauron::Point2DInt point = line.getPoint( 0 );

    std::stringstream ss;
    if ( txt.size() )
        ss << txt;
    cvPutText( im, ss.str().c_str(), cvPoint( point.X() + 3, point.Y() + 10 ), &font, CV_RGB( r, g, b ) );
}
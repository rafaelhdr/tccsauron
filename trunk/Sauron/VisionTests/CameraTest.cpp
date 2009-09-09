#include <iostream>
#include <ctime>
#include <sstream>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "../Vision/Image.h"
#include "../Vision/VerticalLineConvolutionOperator.h"
#include "../Vision/VerticalProjectionDetector.h"
#include "../Vision/Camera.h"
#include "../Vision/ProjectionTracker.h"


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
    cvPutText( im, ss.str().c_str(), cvPoint( point.X() + 3, point.Y() ), &font, CV_RGB( r, g, b ) );
}


int testCamera()
{
    sauron::Camera camera;
    camera.setSize( 640, 480 );

    sauron::Image image( 640, 480, 8, sauron::Pixel::PF_RGB );
    sauron::Image original( image );

    cvNamedWindow( "Sobel", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "Final", CV_WINDOW_AUTOSIZE );

    CvFont font;
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0 );

    clock_t sobelStartTime;
    clock_t lineStartTime;
    clock_t trackStartTime;

    clock_t sobelMeanTime = 0;
    clock_t lineMeanTime  = 0;
    clock_t trackMeanTime = 0;

    clock_t fpsStartTime = clock();

    unsigned int meanCount   = 0;
    unsigned int framesCount = 0;

    sauron::ProjectionTracker               tracker;
    sauron::VerticalProjectionDetector      detector;
    sauron::VerticalLineConvolutionOperator conv;

    sauron::ProjectionVector projections;
    sauron::ProjectionVector projectionsTracked;

    char key = 0;
    while ( key != 'q' && key != 'Q' && key != 27 )
    {
        camera.getFrame( image );
        original = image;
        image.convertToGray();

        sobelStartTime = clock();
        conv.convolve( image );
        sobelMeanTime += clock() - sobelStartTime;

        cvShowImage( "Sobel", image );

        image.convertToGray();
        
        lineStartTime = clock();
        detector.detect( original, image, projections );
        lineMeanTime += clock() - lineStartTime;

        for ( register unsigned int i = 0; i < projections.size(); ++i )
            drawProjection( original, projections[ i ], font, 255, 0, 0 );

        trackStartTime = clock();
        tracker.track( projections, projectionsTracked );
        trackMeanTime += clock() - trackStartTime;

        std::vector< sauron::uint > ids = tracker.debug_getTrackedIDs();

        for ( register unsigned int i = 0; i < projectionsTracked.size(); ++i )
        {
            std::stringstream ss;
            ss << ids[i];
            drawProjection( original, projectionsTracked[ i ], font, 255, 255, 255, ss.str() );
        }

        ++meanCount;
        if ( clock() - fpsStartTime > CLOCKS_PER_SEC )
        {
            double total = (double)(sobelMeanTime + lineMeanTime + trackMeanTime) / meanCount;

            std::cout << "\r";
            std::cout << "Sobel: "   << (double)sobelMeanTime / meanCount;
            std::cout << "  Lines: " << (double)lineMeanTime  / meanCount;
            std::cout << "  Track: " << (double)trackMeanTime / meanCount;
            std::cout << "  Total: " << total;
            std::cout << "  FPS: " << 1000.0 / total;
            fpsStartTime = clock();
        }
        else
            ++framesCount;

        
        cvShowImage( "Final", original );
        key = (char)cvWaitKey( 1 );     
    }

    cvDestroyAllWindows();

    return 0;
}
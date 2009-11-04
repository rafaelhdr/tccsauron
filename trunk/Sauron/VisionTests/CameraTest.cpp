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

extern void drawProjection( sauron::Image &im, const sauron::Projection &proj, CvFont &font, byte r, byte g, byte b, std::string txt = "" );

int testCamera()
{
    const sauron::uint imageScale  = 2;
    const sauron::uint imageWidth  = 320;
    const sauron::uint imageHeight = 240;
    const sauron::uint finalImageWidth  = imageWidth  * imageScale;
    const sauron::uint finalImageHeight = imageHeight * imageScale;

    sauron::Camera camera;
    camera.setSize( finalImageWidth, finalImageHeight );

    sauron::Image image( finalImageWidth, finalImageHeight, 8, sauron::Pixel::PF_RGB );
    sauron::Image original( image );

    //cvNamedWindow( "Sobel", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "Final", CV_WINDOW_AUTOSIZE );

    CvFont font;
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0 );

    clock_t sobelStartTime;
    clock_t lineStartTime;
    clock_t trackStartTime;
    clock_t cameraStartTime;

    clock_t sobelMeanTime = 0;
    clock_t lineMeanTime  = 0;
    clock_t trackMeanTime = 0;
    clock_t cameraMeanTime = 0;

    clock_t fpsStartTime = clock();

    unsigned int meanCount   = 0;
    unsigned int framesCount = 0;

    sauron::ProjectionTracker               tracker;
    sauron::VerticalProjectionDetector      detector;
    sauron::VerticalLineConvolutionOperator conv;

    sauron::ProjectionVector projections;
    sauron::ProjectionVector projectionsTracked;

    sauron::Image fromfile( "0_step_0.bmp" );

    char key = 0;
    while ( key != 'q' && key != 'Q' && key != 27 )
    {
        cameraStartTime = clock();
        camera.getFrame( image );
        cameraMeanTime += clock() - cameraStartTime;
        
        image = fromfile;
        original = image;
        image.convertToGray();

        sobelStartTime = clock();
        conv.convolve( image );
        sobelMeanTime += clock() - sobelStartTime;

        //cvShowImage( "Sobel", image );

        image.convertToGray();
        
        lineStartTime = clock();
        detector.detect( original, image, projections );
        lineMeanTime += clock() - lineStartTime;

        for ( register unsigned int i = 0; i < projections.size(); ++i )
            drawProjection( original, projections[ i ], font, 255, 0, 0 );

        original.save( "projections.jpg" );

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

        //std::cin.get();
    }

    cvDestroyAllWindows();

    return 0;
}
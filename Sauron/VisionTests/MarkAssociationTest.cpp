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
#include "../Vision/Mark.h"
#include "../Vision/MarkPersistenceManager.h"
#include "../Vision/MarkAssociator.h"

extern void drawProjection( sauron::Image &im, const sauron::Projection &proj, CvFont &font, byte r, byte g, byte b, std::string txt = "" );

int testMarkAssociation()
{
    const sauron::uint imageScale  = 1;
    const sauron::uint imageWidth  = 320;
    const sauron::uint imageHeight = 240;
    const sauron::uint finalImageWidth  = imageWidth  * imageScale;
    const sauron::uint finalImageHeight = imageHeight * imageScale;

    sauron::Camera camera;
    camera.setSize( finalImageWidth, finalImageHeight );

    sauron::Image image( finalImageWidth, finalImageHeight, 8, sauron::Pixel::PF_RGB );
    sauron::Image original( image );

    cvNamedWindow( "Final", CV_WINDOW_AUTOSIZE );

    CvFont font;
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0 );

    clock_t sobelStartTime;
    clock_t lineStartTime;
    clock_t trackStartTime;
    clock_t associationStartTime;

    clock_t sobelMeanTime = 0;
    clock_t lineMeanTime  = 0;
    clock_t trackMeanTime = 0;
    clock_t associationMeanTime = 0;

    clock_t fpsStartTime = clock();

    unsigned int meanCount   = 0;
    unsigned int framesCount = 0;

    sauron::ProjectionTracker               tracker;
    sauron::VerticalProjectionDetector      detector;
    sauron::VerticalLineConvolutionOperator conv;
    sauron::MarkAssociator                  associator;

    sauron::ProjectionVector projections;
    sauron::ProjectionVector projectionsTracked;
    sauron::MarkVector       marksAssociated;
    
    sauron::MarkVector       marks;
    sauron::MarkPersistenceManager::loadFromFile( "marks.map", marks );
    associator.loadMarks( marks );

    char key = 0;
    while ( key != 'q' && key != 'Q' && key != 27 )
    {
        camera.getFrame( image );
        original = image;
        image.convertToGray();

        sobelStartTime = clock();
        conv.convolve( image );
        sobelMeanTime += clock() - sobelStartTime;

        image.convertToGray();
        
        lineStartTime = clock();
        detector.detect( original, image, projections );
        lineMeanTime += clock() - lineStartTime;

        for ( register unsigned int i = 0; i < projections.size(); ++i )
            drawProjection( original, projections[ i ], font, 255, 0, 0 );

        trackStartTime = clock();
        tracker.track( projections, projectionsTracked );
        trackMeanTime += clock() - trackStartTime;

        associationStartTime = clock();
        associator.associateMarks( projectionsTracked, marksAssociated, projections );
        associationMeanTime += clock() - associationStartTime;

        //std::vector< sauron::uint > ids = tracker.debug_getTrackedIDs();

        for ( register unsigned int i = 0; i < marksAssociated.size(); ++i )
        {
            drawProjection( original, projections[ i ], font, 255, 255, 255, marksAssociated[i].getDescription() );
            //std::cout << "Mark[ "<< framesCount << "]: " << marksAssociated[i].getDescription() << std::endl;
        }
        std::cout << std::endl;

        ++meanCount;
        if ( clock() - fpsStartTime > CLOCKS_PER_SEC )
        {
            double total = (double)(sobelMeanTime + lineMeanTime + trackMeanTime + associationMeanTime) / meanCount;

            std::system( "cls" );
            std::cout << "Sobel: "   << (double)sobelMeanTime / meanCount << std::endl;
            std::cout << "Lines: " << (double)lineMeanTime  / meanCount << std::endl;
            std::cout << "Track: " << (double)trackMeanTime / meanCount << std::endl;
            std::cout << "Marks: " << (double)associationMeanTime / meanCount << std::endl;
            std::cout << "Total: " << total << std::endl;
            std::cout << "FPS: " << 1000.0 / total << std::endl;
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
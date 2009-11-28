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
#include "../Vision/CoordinateConverter.h"
#include "../Common/Pose.h"

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

    sauron::Pose pose;
    pose.X()     = 4320.0;
    pose.Y()     = 200.0;
    pose.Theta() = 3.1415;//3.1415;

    unsigned int meanCount   = 0;
    unsigned int framesCount = 0;

    //sauron::ProjectionTracker               tracker;
    sauron::VerticalProjectionDetector      detector;
    sauron::VerticalLineConvolutionOperator conv;
    sauron::MarkAssociator                  associator;

    sauron::ProjectionVector projections;
    sauron::ProjectionVector projectionsTracked;
    sauron::ProjectionVector projectionsAssociated;
    sauron::MarkVector       marksAssociated;
    
    sauron::MarkVector       marks;
    sauron::MarkPersistenceManager::loadFromFile( "marks.map", marks );
    associator.loadMarks( marks );

    char key = 0;
    while ( key != 'q' && key != 'Q' && key != 27 )
    {
        if ( !camera.getFrame( image ) )
            continue;

        original = image;
        image.convertToGray();

        sobelStartTime = clock();
        conv.convolve( image );
        sobelMeanTime += clock() - sobelStartTime;

        image.convertToGray();
        
        lineStartTime = clock();
        detector.detect( original, image, projections );
        lineMeanTime += clock() - lineStartTime;

        //trackStartTime = clock();
        //tracker.track( projections, projectionsTracked );
        //trackMeanTime += clock() - trackStartTime;

        for ( register unsigned int i = 0; i < projections.size(); ++i )
        {
            std::stringstream ss;
            ss << projections[i].getDiscretizedLine().getMeanX();
            drawProjection( original, projections[ i ], font, 255, 0, 0, ss.str() );
        }

        associationStartTime = clock();
        associator.associateMarks( projections, pose, marksAssociated, projectionsAssociated );
        associationMeanTime += clock() - associationStartTime;

        //std::vector< sauron::uint > ids = tracker.debug_getTrackedIDs();

        for ( register unsigned int i = 0; i < marksAssociated.size(); ++i )
        {
            drawProjection( original, projectionsAssociated[ i ], font, 0, 255, 0, marksAssociated[i].getDescription() );
            //std::cout << "Mark[ "<< framesCount << "]: " << marksAssociated[i].getDescription() << std::endl;
        }
        std::cout << std::endl;

        for ( register int i = 0; i < marks.size(); ++i )
        {
            double camX = marks[i].getPosition().X() - pose.X();
            double camY = marks[i].getPosition().Y() - pose.Y();
            double rotCamX = camX * cos( pose.Theta() ) - camY * sin( pose.Theta() );
            double rotCamY = camX * sin( pose.Theta() ) + camY * cos( pose.Theta() );
            double posU = sauron::CoordinateConverter::Wordl2Cam_U( -rotCamX, rotCamY );

            if ( posU >= 0 && posU <= original.getWidth() )
            {
                for ( int h = 0; h < original.getHeight(); ++h )
                    original( posU, h ).set( 0, 0, 255 );        

                std::stringstream ss;
                ss << marks[i].getDescription();
                cvPutText( original, ss.str().c_str(), cvPoint( (int)posU - ss.str().size() / 2 * 5, 10 * (i+1) ), &font, CV_RGB( 0, 0, 255 ) );

                std::stringstream ss2;
                ss2 << posU;
                cvPutText( original, ss2.str().c_str(), cvPoint( (int)posU - ss2.str().size() / 2 * 7, original.getHeight() - 20 ), &font, CV_RGB( 0, 0, 255 ) );
            }
        }

        ++meanCount;
        if ( clock() - fpsStartTime > CLOCKS_PER_SEC )
        {
            double total = (double)(sobelMeanTime + lineMeanTime + trackMeanTime + associationMeanTime) / meanCount;

            std::system( "cls" );
            std::cout << "Sobel: "   << (double)sobelMeanTime / meanCount << std::endl;
            std::cout << "Lines: " << (double)lineMeanTime  / meanCount << std::endl;
            //std::cout << "Track: " << (double)trackMeanTime / meanCount << std::endl;
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
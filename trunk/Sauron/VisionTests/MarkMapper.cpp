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


extern void drawProjection( sauron::Image &im, const sauron::Projection &proj, CvFont &font, byte r, byte g, byte b, std::string txt = "" );


int testMarkMapper()
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

    sauron::ProjectionTracker               tracker;
    sauron::VerticalProjectionDetector      detector;
    sauron::VerticalLineConvolutionOperator conv;

    sauron::ProjectionVector projections;
    sauron::ProjectionVector projectionsTracked;

    sauron::MarkVector marks;

    std::string validChoices = "SsQqAaNn\n";

    bool run = true;
    while ( run )
    {
        camera.getFrame( image );
        original = image;
        image.convertToGray();

        conv.convolve( image );
        image.convertToGray();
        detector.detect( original, image, projections );
        tracker.track( projections, projectionsTracked );

        for ( register unsigned int i = 0; i < projectionsTracked.size(); ++i )
        {
            std::stringstream ss;
            ss << i;
            drawProjection( original, projectionsTracked[ i ], font, 255, 255, 255, ss.str() );
        }
        
        cvShowImage( "Final", original );
        cvWaitKey(1);

        char choice = 0;
        std::cin.clear();
        do
        {
            std::cout << "(S)elect   (N)ext   (Q)uit   (A)bort   => ";
            std::cin.get( choice );
        }
        while ( validChoices.find( choice ) == std::string::npos );
        
        int projectionToMarkID;
        float x;
        float y;
        std::string description;

        switch ( choice )
        {
            case 'S':
            case 's':
                std::cout << "Projection ID: ";
                std::cin  >> projectionToMarkID;
                std::cout << "Coordinates: ";
                std::cin  >> x >> y;
                std::cout << "Description: ";
                std::cin  >> description;
                marks.push_back( sauron::Mark( sauron::Point2DFloat( x, y ), projectionsTracked[projectionToMarkID].getColorProfile(), description ) );
                std::cout << std::endl << std::endl;
                break;

            case 'N':
            case 'n':
            case '\n':
                break;

            case 'Q':
            case 'q':
                sauron::MarkPersistenceManager::saveToFile( "marks.map", marks );
                
            case 'A':
            case 'a':
                run = false;
                break;
        }

    }

    cvDestroyAllWindows();

    return 0;
}
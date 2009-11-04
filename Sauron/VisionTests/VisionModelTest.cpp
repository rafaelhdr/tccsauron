#include <iostream>
#include <ctime>
#include <sstream>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "../Vision/VisionModel.h"

extern void drawProjection( sauron::Image &im, const sauron::Projection &proj, CvFont &font, byte r, byte g, byte b, std::string txt = "" );

int testVisionModel()
{
    sauron::Image       frame( 320, 240, 8, sauron::Pixel::PF_RGB );
    sauron::VisionModel visionModel;

    CvFont font;
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0 );

    cvNamedWindow( "Frame", -1 );

    clock_t fpsStartTime = clock();
    unsigned int framesCount = 0;

    char key = 0;
    while ( key != 'q' && key != 'Q' && key != 27 )
    {
        //visionModel.getLastFrame( frame );
        visionModel.getLastFrameWithTrackedProjections( frame );
        cvShowImage( "Frame", frame );

        if ( clock() - fpsStartTime > CLOCKS_PER_SEC )
        {
            std::cout << "Main Loop: " << (double)framesCount * CLOCKS_PER_SEC / (double)(clock() - fpsStartTime) << " <=> " << framesCount <<  std::endl;
            framesCount = 0;
            fpsStartTime = clock();
        }
        else
            ++framesCount;

        key = (char)cvWaitKey( 1 );
    }

    cvDestroyAllWindows();

    return 0;
}
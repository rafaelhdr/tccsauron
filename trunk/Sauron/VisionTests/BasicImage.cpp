#include <iostream>
#include "../Vision/Image.h"
#include "../Vision/VerticalLineConvolutionOperator.h"
#include "../Vision/VerticalProjectionDetector.h"

int testBasicImage()
{
    char *filename = "0_step_0.bmp";

    sauron::Image image( filename );
    
    cvNamedWindow( "BasicImage", 2 );
    cvShowImage( "BasicImage", image );
    cvWaitKey();

    sauron::VerticalLineConvolutionOperator conv;
    conv.convolve( image );

    image.save( "sobel.jpg" );

    cvShowImage( "BasicImage", image );
    cvWaitKey();

    image.convertToGray();
    sauron::VerticalProjectionDetector detector;
    //detector.detect( image );

    cvShowImage( "BasicImage", image );
    cvWaitKey();

    sauron::Image original( filename );
    for ( register sauron::uint i = 0; i < image.getWidth(); ++i )
    {
        for ( register sauron::uint j = 0; j < image.getHeight(); ++j )
        {
            if ( image(i, j).R() || image(i, j).G() || image(i, j).B() )
            {
                original(i, j).R() = image(i, j).R();
                original(i, j).G() = image(i, j).G();
                original(i, j).B() = image(i, j).B();
            }
        }
    }
    cvShowImage( "BasicImage", original );
    cvWaitKey();
    original.save( "output.jpg" );

    cvDestroyAllWindows();
    
    return 0;
}

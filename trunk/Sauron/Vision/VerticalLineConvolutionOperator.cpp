#include "VerticalLineConvolutionOperator.h"

#define VERTICAL_SOBEL_USE_COLORS   0

namespace sauron
{

VerticalLineConvolutionOperator::VerticalLineConvolutionOperator()
{
}

VerticalLineConvolutionOperator::~VerticalLineConvolutionOperator()
{
}

void VerticalLineConvolutionOperator::convolve( Image &im )
{
    cvSmooth( im, im, CV_GAUSSIAN);

#if VERTICAL_SOBEL_USE_COLORS
    int sumR = 0;
    int sumG = 0;
    int sumB = 0;
#else 
    int sumGray = 0;
#endif

    //const int kernelValues[] = 
    //{
    //    1, 1, 0, -1, -1,
    //    1, 1, 0, -1, -1,
    //    1, 1, 0, -1, -1,
    //    1, 1, 0, -1, -1,
    //    1, 1, 0, -1, -1,
    //    1, 1, 0, -1, -1,
    //    1, 1, 0, -1, -1,
    //};

    //const int kernelValues[] = 
    //{
    //    2, 2, 0, -2, -2,
    //    2, 2, 0, -2, -2,
    //};

    const int kernelValues[] = 
    {
        2, 0, -2,
        2, 0, -2,
        2, 0, -2,
        2, 0, -2,
        2, 0, -2,
        2, 0, -2,
        2, 0, -2,
    };

    const uint matrixWidth  = 3;  // Remember to change each time the kernelValues are modified
    const uint matrixHeight = 3;  // Remember to change each time the kernelValues are modified
    const uint matrixCenterWidth  = (matrixWidth) >> 1;
    const uint matrixCenterHeight = (matrixHeight) >> 1;
    
    const uint imageWidth   = im.getWidth()  - matrixCenterWidth;
    const uint imageHeight  = im.getHeight() - matrixCenterHeight;

    const float scale = 0.1f;

    cvScale( im, im, 0.5 );

#if !VERTICAL_SOBEL_USE_COLORS
    im.convertToGray();
#endif

    Image buffer( im );

    for ( register unsigned int i = matrixCenterWidth; i < imageWidth; ++i )
    {
        for ( register unsigned int j = matrixCenterHeight; j < imageHeight; ++j )
        {
#if VERTICAL_SOBEL_USE_COLORS
            sumR = 0;
            sumG = 0;
            sumB = 0;
#else
            sumGray = 0;
#endif
      
            int unfinishedIndex = 0;
            for( register unsigned int r = 0; r < matrixHeight; ++r )
            {
                unfinishedIndex = matrixWidth * r;
                for ( register unsigned int c = 0; c < matrixWidth; ++c )
                {
                    int kernelValue = kernelValues[ unfinishedIndex + c];
                    Pixel pixel = buffer(i - matrixCenterWidth + c, j - matrixCenterHeight + r);
#if VERTICAL_SOBEL_USE_COLORS
                    sumR += (pixel.R()) * kernelValue * scale;
                    sumG += (pixel.G()) * kernelValue * scale;
                    sumB += (pixel.B()) * kernelValue * scale;
#else
                    sumGray += pixel.Gray() * kernelValue; //* scale;
#endif
                }
            }

#if VERTICAL_SOBEL_USE_COLORS
            byte rgb[3];
            rgb[0] = (byte)(sumR < 0 ? (-sumR > 255 ? 255 : -sumR) : (sumR > 255 ? 255 : sumR));
            rgb[1] = (byte)(sumG < 0 ? (-sumG > 255 ? 255 : -sumG) : (sumG > 255 ? 255 : sumG));
            rgb[2] = (byte)(sumB < 0 ? (-sumB > 255 ? 255 : -sumB) : (sumB > 255 ? 255 : sumB));

            im(i, j).set( rgb[0], rgb[1], rgb[2] );
#else           
            im(i, j).Gray() = (byte)(sumGray < 0 ? (-sumGray > 255 ? 255 : -sumGray) : (sumGray > 255 ? 255 : sumGray));
#endif
        }
    }
}

}   // namespace sauron
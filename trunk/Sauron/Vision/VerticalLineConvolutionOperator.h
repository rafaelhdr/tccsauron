#ifndef __VERTICAL_SOBEL_CONVOLUTION_OPERATOR_H__
#define __VERTICAL_SOBEL_CONVOLUTION_OPERATOR_H__

#include "IConvolutionOperator.h"
#include "Image.h"

namespace sauron
{

class VerticalLineConvolutionOperator : public IConvolutionOperator
{
    public:
        VerticalLineConvolutionOperator();
        ~VerticalLineConvolutionOperator();

        void convolve( Image &im );
};

}  // namespace sauron

#endif // __VERTICAL_SOBEL_CONVOLUTION_OPERATOR_H__
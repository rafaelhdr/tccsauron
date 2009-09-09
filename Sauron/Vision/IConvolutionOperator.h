#ifndef __CONVOLUTION_OPERATOR_H__
#define __CONVOLUTION_OPERATOR_H__

#include "Image.h"

namespace sauron
{

class IConvolutionOperator
{
    public:
        virtual void convolve( Image &im ) = 0;        
};
    
}; // namespace sauron


#endif /* __CONVOLUTION_OPERATOR_H__ */

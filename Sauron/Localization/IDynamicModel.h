#ifndef __DYNAMIC_H__
#define __DYNAMIC_H__

#include "Pose.h"
#include "Matrix.h"

namespace sauron
{

class IDynamicModel
{
    public:
        virtual void updateModel( const Pose &last, 
                                  pose_t &fValue, Model &dynModel, Covariance &dynNoise ) = 0;
};

}   // namespace sauron

#endif  // __DYNAMIC_H__
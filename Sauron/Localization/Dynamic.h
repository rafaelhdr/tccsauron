#ifndef __DYNAMIC_H__
#define __DYNAMIC_H__

#include "Pose.h"
#include "Matrix.h"

namespace sauron
{

class Dynamic
{
    public:
        virtual void updateModel( const Pose &last, 
                                  Matrix &dynModel, Matrix &dynNoise ) = 0;
};

}   // namespace sauron

#endif  // __DYNAMIC_H__
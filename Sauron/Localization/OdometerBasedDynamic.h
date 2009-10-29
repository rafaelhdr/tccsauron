#ifndef __ODOMETER_BASED_DYNAMIC_H__
#define __ODOMETER_BASED_DYNAMIC_H__

#include "IDynamicModel.h"

namespace sauron
{

class OdometerBasedDynamic : public IDynamicModel
{
    public:
        OdometerBasedDynamic();
        ~OdometerBasedDynamic();

        void updateModel( const Pose &last, 
                          pose_t &fValue, Model &dynModel, Covariance &dynNoise );
};

}   // namespace sauron

#endif  // __ODOMETER_BASED_DYNAMIC_H__
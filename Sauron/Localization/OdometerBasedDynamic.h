#ifndef __ODOMETER_BASED_DYNAMIC_H__
#define __ODOMETER_BASED_DYNAMIC_H__

#include "Dynamic.h"

namespace sauron
{

class OdometerBasedDynamic : public Dynamic
{
    public:
        OdometerBasedDynamic(void);
        ~OdometerBasedDynamic(void);

        void updateModel( const Pose &last, 
                          Matrix &dynModel, Matrix &dynNoise );
};

}   // namespace sauron

#endif  // __ODOMETER_BASED_DYNAMIC_H__
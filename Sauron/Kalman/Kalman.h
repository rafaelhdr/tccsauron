#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include <cv.h>
#include <cxcore.h>
#include "Pose.h"
#include "Matrix.h"

namespace sauron
{

class Kalman
{
    public:
        Kalman();

        void setStateTransisitionModel( const Matrix &A );
        /*void setControlMode( const Matrix &B );*/
        void setMeasurementModel( const Matrix &H );
        void setProcessNoise( double noise );
        void setMeasureNoise( double noise );

        void predict( Pose &pose );
        void correct( Pose &pose /*Measure goes here*/ );

    private:
        CvKalman *m_kalman;
};

}   // namespace sauron

#endif // _KALMAN_FILTER_H_
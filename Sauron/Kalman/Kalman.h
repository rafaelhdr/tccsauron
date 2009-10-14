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
        ~Kalman();

        void setStateTransisitionModel( const Matrix &A );
        /*void setControlMode( const Matrix &B );*/
        void setMeasurementModel( const Matrix &H );
        void setProcessNoise( double noise );
        void setMeasureNoise( double noise );

        void predict( Pose &preliminar );
        void correct( const Measure &mes, Pose &final );

    private:
        CvKalman *m_kalman;
        CvMat    *m_auxiliarMatrix;
        CvMat    *m_auxiliarMatrix2;
};

}   // namespace sauron

#endif // _KALMAN_FILTER_H_
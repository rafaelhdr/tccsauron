#ifndef __EXTENDED_KALMAN_FILTER_H__
#define __EXTENDED_KALMAN_FILTER_H__

#include "IKalmanFilter.h"

namespace sauron
{

class ExtendedKalmanFilter : public IKalmanFilter
{
    public:
        ExtendedKalmanFilter();
        ~ExtendedKalmanFilter();

        void getPrioriEstimate( const Matrix &fValue, const Model &F, const Covariance &Q, 
                                Pose &estimate,  Covariance &P );

        void getPosterioriEstimate( const Measure &z,		// a medida obtida
									const Matrix &hValue,	// a medida esperada
									const Model &H,			// matriz de observação
									const Covariance &R,	// covariância
                                    Pose &estimate,			// in: a estimativa a priori da posição
															// out: a estimativa a posteriori
									Covariance &P);			// in: a covariância a priori
															// out: a covariãncia a posteriori
};

}   //  namespace sauron

#endif  // __EXTENDED_KALMAN_FILTER_H__
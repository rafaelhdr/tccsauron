#ifndef __EXTENDED_KALMAN_FILTER_H__
#define __EXTENDED_KALMAN_FILTER_H__

#include "IKalmanFilter.h"

namespace sauron
{

class ExtendedKalmanFilter : public IKalmanFilter
{
    public:

		ExtendedKalmanFilter()
            : m_latestCovariance(3,3) {
				m_latestCovariance.clear();
		}

		ExtendedKalmanFilter(const Pose& initialPose) 
            : m_latestCovariance(3,3),
              m_latestEstimate(initialPose) {
				  m_latestCovariance.clear();
		}

		ExtendedKalmanFilter(const Pose& initialPose, const Covariance& initialCovariance) 
            : m_latestEstimate(initialPose), 
              m_latestCovariance(initialCovariance) {}

        ~ExtendedKalmanFilter();

        void predict( const Matrix &fValue, const Model &F, const Covariance &Q);

		// Atualiza as estimativas com os dados de um sensor.
		// As seguintes restrições quanto as tamanhos das matrizes devem ser obedecidos:
		// H.size2() == 3
		// R.size1() == R.size2() == H.size1()
		// z.size1() == hValue.size1()
		// z.size2() == hValue.size2()
		// z.size1() == H.size1()
        void update( const Measure &z,		// a medida obtida
					 const Matrix &hValue,	// a medida esperada
					 const Model &H,		// matriz de observação
					 const Covariance &R );	// covariância

		inline Pose getLatestEstimate()                     { return m_latestEstimate; }
		inline void setLatestEstimate(const Pose& estimate) { m_latestEstimate = estimate; m_latestCovariance.clear(); }

		inline Covariance getLatestCovariance()                       { return m_latestCovariance; }
		inline void setLatestCovariance(const Covariance& covariance) { m_latestCovariance = covariance; }

    private:
	    Pose m_latestEstimate;
	    Covariance m_latestCovariance;

};

}   //  namespace sauron

#endif  // __EXTENDED_KALMAN_FILTER_H__
#include "ExtendedKalmanFilter.h"
#include <boost/numeric/ublas/io.hpp>
#include "log.h"
#include "MathHelper.h"

#define EKF_LOG(level) FILE_LOG(level) << "EKF: "
#define PREDICT_LOG(level) EKF_LOG(level) << "predict: "
#define UPDATE_LOG(level) EKF_LOG(level) << "update: "

namespace sauron
{
	void ExtendedKalmanFilter::predict( const Matrix &fValue, const Model &F, const Covariance &Q)
	{
		using namespace boost::numeric::ublas;

		PREDICT_LOG(logDEBUG4) << "predict fValue = " << fValue;
		PREDICT_LOG(logDEBUG4) << "predict F = " << F;
		PREDICT_LOG(logDEBUG4) << "predict Q = " << Q;
		Matrix temp1(3,3);	

		// Pk = F*P*F'+Q
		temp1 = prod(F,m_latestCovariance);
		temp1 = prod(temp1, boost::numeric::ublas::trans(F));

		m_latestCovariance = temp1 + Q;

		//estimate = prod(F,estimate);
		m_latestEstimate.X() = fValue(0,0);
		m_latestEstimate.Y() = fValue(1,0);
		m_latestEstimate.setTheta(fValue(2,0));
		PREDICT_LOG(logDEBUG3) << "predict estimativa: " << m_latestEstimate;
		PREDICT_LOG(logDEBUG3) << "predict covariância: " << m_latestCovariance;
	}

	void ExtendedKalmanFilter::initCovariance()
	{
		m_latestCovariance.clear();
		m_latestCovariance(0,0) = 100;
		m_latestCovariance(1,1) = 15;
		m_latestCovariance(2,2) = 0.3;
	}


	// As seguintes restrições quanto as tamanhos das matrizes devem ser obedecidos:
	// H.size2() == 3
	// R.size1() == R.size2() == H.size1()
	// z.size1() == hValue.size1()
	// z.size2() == hValue.size2()
	// z.size1() == H.size1()
	void ExtendedKalmanFilter::update( const Measure &z, const Matrix &hValue, const Model &H, const Covariance &R )
	{
		using namespace boost::numeric::ublas;
		UPDATE_LOG(logDEBUG2) << "z = " << z;
		UPDATE_LOG(logDEBUG2) << "hValue = " << hValue;
		UPDATE_LOG(logDEBUG2) << "H = " << H;
		UPDATE_LOG(logDEBUG2) << "R = " << R;
		Matrix K( H.size2(), H.size1() ); // é nessa ordem mesmo ( 2 -> 1 )

		
		Matrix yTemp( 3, 1 );

		// Kk = P*C'*inv(H*P*H' + R)
		Matrix temp1 = prod(H,m_latestCovariance);
		UPDATE_LOG(logDEBUG3) << "H * m_latestCovariance = " << temp1;
		temp1 = prod(temp1, boost::numeric::ublas::trans(H));
		UPDATE_LOG(logDEBUG3) << "H * m_latestCovariance * H' = " << temp1;
		temp1 = temp1 + R;
		UPDATE_LOG(logDEBUG3) << "H * m_latestCovariance * H' + R = " << temp1;

		Matrix temp2(temp1.size1(), temp1.size2());
		algelin::InvertMatrix(temp1, temp2);// temp2 = inv(H*P*H' + R)
		UPDATE_LOG(logDEBUG3) << "inv(H * m_latestCovariance * H' + R) = " << temp2;

		temp1 = prod(m_latestCovariance, trans(H));
		UPDATE_LOG(logDEBUG3) << "m_latestCovariance * H' = " << temp1;
		K = prod(temp1,temp2);
		
		UPDATE_LOG(logDEBUG2) << "K = m_latestCovariance * H' * inv(H * m_latestCovariance * H' + R)" << K;

		// xk = x + K*(y - hk(xk,0))
		yTemp = z - hValue;
		UPDATE_LOG(logDEBUG3) << "z - hValue" << yTemp;
		yTemp = prod(K, yTemp);

		UPDATE_LOG(logDEBUG2) << "K * (z - hValue) = " << yTemp;

		m_latestEstimate.X() = m_latestEstimate.X() + yTemp(0,0);
		m_latestEstimate.Y() = m_latestEstimate.Y() + yTemp(1,0);
		m_latestEstimate.Theta() = m_latestEstimate.Theta() + yTemp(2,0);

		// Pk = (I - K*H)*P
		identity_matrix<double> I (3);
		temp1 = prod(K,H);
		temp1 = I - temp1;
		m_latestCovariance = prod(temp1,m_latestCovariance);

		UPDATE_LOG(logDEBUG2) << "estimativa: " << m_latestEstimate;
		UPDATE_LOG(logDEBUG2) << "covariância: " << m_latestCovariance;
	}

}   // namespace sauron
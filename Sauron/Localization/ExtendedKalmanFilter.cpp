#include "ExtendedKalmanFilter.h"
#include <boost/numeric/ublas/io.hpp>
#include "MathHelper.h"


namespace sauron
{

	void ExtendedKalmanFilter::predict( const Matrix &fValue, const Model &F, const Covariance &Q)
	{
		using namespace boost::numeric::ublas;

		Matrix temp1(3,3);	

		// Pk = F*P*F'+Q
		temp1 = prod(F,m_latestCovariance);
		temp1 = prod(temp1, boost::numeric::ublas::trans(F));

		m_latestCovariance = temp1 + Q;

		//estimate = prod(F,estimate);
		m_latestEstimate.X() = fValue(0,0);
		m_latestEstimate.Y() = fValue(1,0);
		m_latestEstimate.setTheta(fValue(2,0));
	}


	void ExtendedKalmanFilter::update( const Measure &z, const Matrix &hValue, const Model &H, const Covariance &R )
	{
		using namespace boost::numeric::ublas;

		Matrix K( H.size2(), H.size1() ); // é nessa ordem mesmo ( 2 -> 1 )

		Matrix temp1( H.size1(), H.size2() );
		Matrix temp2( H.size1(), H.size2() );
		Matrix yTemp( 3, 1 );

		// Kk = P*C'*inv(H*P*H' + R)
		temp1 = prod(H,m_latestCovariance);
		temp1 = prod(temp1, boost::numeric::ublas::trans(H));
		temp1 = temp1 + R;

		algelin::InvertMatrix(temp1, temp2);// temp2 = inv(H*P*H' + R)

		temp1 = prod(m_latestCovariance, trans(H));
		K = prod(temp1,temp2);

		// xk = x + K*(y - hk(xk,0))
		yTemp = z - hValue;
		yTemp = prod(K, yTemp);
		m_latestEstimate.X() = m_latestEstimate.X() + yTemp(0,0);
		m_latestEstimate.Y() = m_latestEstimate.Y() + yTemp(1,0);
		m_latestEstimate.Theta() = m_latestEstimate.Theta() + yTemp(2,0);

		// Pk = (I - K*H)*P
		identity_matrix<double> I (3);
		temp1 = prod(K,H);
		temp1 = I - temp1;
		m_latestCovariance = prod(temp1,m_latestCovariance);
	}

}   // namespace sauron
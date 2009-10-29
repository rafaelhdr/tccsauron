#include "ExtendedKalmanFilter.h"
#include <boost/numeric/ublas/io.hpp>
#include "MathHelper.h"


namespace sauron
{

	ExtendedKalmanFilter::ExtendedKalmanFilter()
	{
		// TODO
	}

	ExtendedKalmanFilter::~ExtendedKalmanFilter()
	{
		// TODO
	}


	void ExtendedKalmanFilter::getPrioriEstimate( const Model &F, const Covariance &Q, 
		Pose &estimate,  Covariance &P )
	{
		using namespace boost::numeric::ublas;

		Matrix temp1(3,3);

		// Pk = F*P*F'+Q
		temp1 = prod(F,P);
		temp1 = prod(temp1, trans(F)); // F*P*F'

		P = temp1 + Q;

		// xhat = f(x, u, 0)
		//estimate = prod(F,estimate);
	}


	void ExtendedKalmanFilter::getPosterioriEstimate( const Measure &z, const Model &H, const Covariance &R, 
		Pose &estimate, Covariance &P )
	{
		using namespace boost::numeric::ublas;

		Matrix K(3,3);

		Matrix temp1(3,3);
		Matrix temp2(3,3);

		// Kk = P*C'*inv(H*P*H' + R)
		temp1 = prod(H,P);
		temp1 = prod(temp1, trans(H));
		temp1 = temp1 + R;

		algelin::InvertMatrix(temp1, temp2);// temp2 = inv(H*P*H' + R)

		temp1 = prod(P, trans(H));
		K = prod(temp1,temp2);

		// xk = x + K*(y - hk(xk,0))
		// temp1 = z - h;
		// estimate = estimate + prod(K, temp1);

		// Pk = (I - K*H)*P
		identity_matrix<double> I (3);
		temp1 = prod(K,H);
		temp1 = I - temp1;
		P = prod(temp1,P);
	}

}   // namespace sauron
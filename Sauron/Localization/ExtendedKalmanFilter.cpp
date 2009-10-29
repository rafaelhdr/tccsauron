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


	void ExtendedKalmanFilter::getPrioriEstimate( const Matrix &fValue, const Model &F, const Covariance &Q, 
		Pose &estimate,  Covariance &P )
	{
		using namespace boost::numeric::ublas;

		Matrix temp1(3,3);
		Matrix xhat(3,1);

		// xhat = estimate
		xhat(0,0) = estimate.X();
		xhat(1,0) = estimate.Y();
		xhat(2,0) = estimate.getTheta();		

		// Pk = F*P*F'+Q
		temp1 = prod(F,P);
		temp1 = prod(temp1, boost::numeric::ublas::trans(F));

		P = temp1 + Q;

		// xhat = f(x, u, 0)
		xhat = prod(fValue,xhat);

		//estimate = prod(F,estimate);
		estimate.X() = xhat(0,0);
		estimate.Y() = xhat(1,0);
		estimate.setTheta(xhat(2,0));
	}


	void ExtendedKalmanFilter::getPosterioriEstimate( const Measure &z, const Matrix &hValue, const Model &H, const Covariance &R, 
		Pose &estimate, Covariance &P )
	{
		using namespace boost::numeric::ublas;

		Matrix K(3,3);

		Matrix temp1(H.size1,H.size2);
		Matrix temp2(H.size1,H.size2);
		Matrix yTemp(3,1);

		// Kk = P*C'*inv(H*P*H' + R)
		temp1 = prod(H,P);
		temp1 = prod(temp1, boost::numeric::ublas::trans(H));
		temp1 = temp1 + R;

		algelin::InvertMatrix(temp1, temp2);// temp2 = inv(H*P*H' + R)

		temp1 = prod(P, trans(H));
		K = prod(temp1,temp2);

		// xk = x + K*(y - hk(xk,0))
		yTemp = z - hValue;
		yTemp = prod(K, yTemp);
		estimate.X() = estimate.X() + yTemp(0,0);
		estimate.Y() = estimate.Y() + yTemp(1,0);
		estimate.setTheta(estimate.getTheta() + yTemp(2,0));

		// Pk = (I - K*H)*P
		identity_matrix<double> I (3);
		temp1 = prod(K,H);
		temp1 = I - temp1;
		P = prod(temp1,P);
	}

}   // namespace sauron
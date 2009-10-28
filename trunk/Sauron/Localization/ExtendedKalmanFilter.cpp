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

		Matrix A(3,3); // Passado como parâmetro pelo modelo de dinâmica - dF/dx|xk-1
		Matrix L(3,3); // Passado como parâmetro pelo modelo de dinâmica - dF/dw|xk-1

		Matrix temp1(3,3);
		Matrix temp2(3,3);


		// Pk = F*P*F'+L*Q*L'
		temp1 = prod(A,P);
		temp1 = prod(temp1, trans(A)); // F*P*F'

		temp2 = prod(L,Q);
		temp2 = prod(temp2, trans(L)); // L*Q*L'

		P = temp1 + temp2;

		// xhat = f(x, u, 0)
		//estimate = prod(F,estimate);
	}


	void ExtendedKalmanFilter::getPosterioriEstimate( const Measure &z, const Model &H, const Covariance &R, 
		Pose &estimate, Covariance &P )
	{
		using namespace boost::numeric::ublas;

		Matrix C(3,3); // Passado como parâmetro pelo modelo de dinâmica - dh/dx|xk
		Matrix M(3,3); // Passado como parâmetro pelo modelo de dinâmica - dh/dv|xk

		Matrix K(3,3);

		Matrix temp1(3,3);
		Matrix temp2(3,3);

		// Kk = P*C'*inv(C*P*C' + C*R*C')
		temp1 = prod(C,P);
		temp1 = prod(temp1, trans(C)); // F*P*F'

		temp2 = prod(M,R);
		temp2 = prod(temp2, trans(M)); // L*Q*L'
		
		temp1 = temp1 + temp2;
		algelin::InvertMatrix(temp1, temp2);// temp2 = inv(C*P*C' + C*R*C')

		temp1 = prod(P, trans(C));
		K = prod(temp1,temp2);

		// xk = x + K*(y - hk(xk,0))
		temp1 = z - H;
		//estimate = estimate + prod(K, temp1);

		// Pk = (I - K*C)*P
		identity_matrix<double> I (3);
		temp1 = prod(K,C);
		temp1 = I - temp1;
		P = prod(temp1,P);
	}

}   // namespace sauron
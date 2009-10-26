#include "MedidaOdometro.h"
#include "Pose.h"

#include <boost/numeric/ublas/matrix.hpp>


namespace sauron 
{

	namespace modeloDinamica
	{
		
		class ModeloDinamica
		{
			private:
				MedidaOdometro medida_anterior;
				MedidaOdometro nova_medida;
				Pose posicao_estimada;

				pose_t calculaX();
				pose_t calculaY();
				pose_t calculaTheta();

				double getVarianciaLinear();
				double getVarianciaAngular();
				
				double influLinearLinear;
				double influLinearAngular;
				double influAngularLinear;
				double influAngularAngular;

				void init()
				{
					influLinearLinear = 0.033;
					influLinearAngular = 0.001;
					influAngularLinear = 0;
					influAngularAngular = 0.035;
				}

			


			public:
				ModeloDinamica(Pose posicao_inicial);

				ModeloDinamica(Pose posicao_inicial, MedidaOdometro medida_inicial);

				Pose getNovaPosicao(MedidaOdometro nova_medida);
				void atualizaPosicao(Pose nova_estimada);

				boost::numeric::ublas::matrix<double> getQ();


		};



	}

}
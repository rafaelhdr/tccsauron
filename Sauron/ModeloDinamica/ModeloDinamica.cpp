// ModeloDinamica.cpp : Defines the exported functions for the DLL application.
//
#include "ModeloDinamica.h"
#include "MathHelper.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>



namespace sauron
{

	namespace modeloDinamica
	{


			ModeloDinamica::ModeloDinamica(Pose posicao_inicial)
			{
				init();
				posicao_estimada = posicao_inicial;
			    medida_anterior = nova_medida = MedidaOdometro(0 ,0); // medida_anterior(0, 0, 0);
			}

			ModeloDinamica::ModeloDinamica(Pose posicao_inicial, MedidaOdometro medida_inicial)
			{
				init();
				posicao_estimada = posicao_inicial; // qual a diferença para isto: posicao_anterior(posicao_inicial);
				medida_anterior = nova_medida = medida_inicial; // medida_anterior(medida_inicial);
			}



		Pose ModeloDinamica::getNovaPosicao(MedidaOdometro nova_medida)
		{
			 medida_anterior = this->nova_medida;
			 this->nova_medida = nova_medida;
			 posicao_estimada = Pose(calculaX(), calculaY(), calculaTheta());
			 
			 return posicao_estimada;
		}
		


		pose_t ModeloDinamica::calculaX()
		{
			/* x(n-1) + delta distancia * cos (teta médio)
			/* dúvida: teta médio anterior ou atual ? */
			return posicao_estimada.X() + nova_medida.minus(medida_anterior).getDistance()*::cos(medida_anterior.getTheta());
		}

		pose_t ModeloDinamica::calculaY()
		{
			return posicao_estimada.Y() + nova_medida.minus(medida_anterior).getDistance()*::sin(medida_anterior.getTheta());
		}

		pose_t ModeloDinamica::calculaTheta()
		{
			/*
			TODO: testar se vem o theta acumulado
			odometro traz o delta do theta ou ele acumula o resultado ? 
			 acho que ele acumula, então basta retornar
             se não, o theta médio passa a ser: medida_anterior.getTheta() + nova_medida.getTheta();
			*/
			return nova_medida.getTheta();

		}

		void ModeloDinamica::atualizaPosicao(Pose posicao_estimada)
		{
			this->posicao_estimada = posicao_estimada;
		}



		double ModeloDinamica::getVarianciaLinear()
		{
			double dm = nova_medida.minus(medida_anterior).getDistance();
			double thetaM = nova_medida.getTheta();
			return (dm*influLinearLinear)*(dm*influLinearLinear) + (thetaM*influAngularLinear)*(thetaM*influAngularLinear);
		}
		
		double ModeloDinamica::getVarianciaAngular()
		{
			double dm = nova_medida.minus(medida_anterior).getDistance();
			double thetaM = nova_medida.getTheta();
			return (thetaM*influAngularAngular)*(thetaM*influAngularAngular) + (dm*influLinearAngular)*(dm*influLinearAngular);
		}

			
		boost::numeric::ublas::matrix<double> ModeloDinamica::getQ()
		{

			using namespace boost::numeric::ublas;


			/*
										(X)T = transposta de X 
			 Q(n) = Fobs(n) . Qobs(n) . (Fobs(n))T
			
			*/

			double cosTheta = ::cos(posicao_estimada.getTheta());
			double senTheta = ::sin(posicao_estimada.getTheta());
			double varLinear = getVarianciaLinear();
			double varAngular = getVarianciaAngular();

			matrix<double> Q(3, 3);
			Q(0,0) = varLinear*cosTheta*cosTheta;
			Q(0,1) = varLinear*cosTheta*senTheta;
			Q(0,2) = 0;

			Q(1,0) = Q(0,1);
			Q(1,1) = varLinear*senTheta*senTheta;
			Q(1,2) = 0;

			Q(2,0) = 0;
			Q(2,1) = 0;
			Q(2,2) = varAngular;
			
			return Q;
		}

		
	}



}




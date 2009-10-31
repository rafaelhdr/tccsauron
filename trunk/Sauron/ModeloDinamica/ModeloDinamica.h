#pragma once
#include "MedidaOdometro.h"
#include "Pose.h"
#include "IDynamicModel.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <Aria.h>


namespace sauron 
{

	namespace modeloDinamica
	{

		class ModeloDinamica : public IDynamicModel
		{


		public:
			ModeloDinamica(ArRobot& robot);
			ModeloDinamica(Pose posicaoEstimada, ArRobot& robot);
			void updateModel( const Pose &last, 
				Matrix &fValue, Model &dynModel, Covariance &dynNoise );

		private:

			MedidaOdometro medidaOdometro;

			Pose posicaoEstimada;

			pose_t calculaX();
			pose_t calculaY();
			pose_t calculaTheta();

			double getVarianciaLinear();
			double getVarianciaAngular();

			double influLinearLinear;
			double influLinearAngular;
			double influAngularLinear;
			double influAngularAngular;

			double normalizaTheta(double valor);

			void init()
			{
				influLinearLinear = 0.033;
				influLinearAngular = 0.001;
				influAngularLinear = 0;
				influAngularAngular = 0.035;
			}

			void atualizaCovariancia(Covariance &dynNoise);

			void atualizaModel(Model &dynModel);

			void atualizaFValue(Matrix &fValue);			
		};
	}
}
#pragma once
#include "MedidaOdometro.h"
#include "Pose.h"
#include "IDynamicModel.h"
#include "ILocalizationManager.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
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
            ~ModeloDinamica();

            void setLocalizationManager( ILocalizationManager &locManager );

            void mainLoop();

        private:
            void updateModel( Matrix &fValue, Model &dynModel, Covariance &dynNoise );

            double normalizaTheta(double valor);

			void init();

			void atualizaCovariancia(Covariance &dynNoise);
			void atualizaModel(Model &dynModel);
			void atualizaFValue(Matrix &fValue);

            pose_t calculaX();
			pose_t calculaY();
			pose_t calculaTheta();

			double getVarianciaLinear();
			double getVarianciaAngular();

		private:
			Pose           posicaoEstimada;
            MedidaOdometro medidaOdometro;

            ILocalizationManager    *m_localizationManager;

			double influLinearLinear;
			double influLinearAngular;
			double influAngularLinear;
			double influAngularAngular;

            boost::thread   m_thread;						
		};
	}
}
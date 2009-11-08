// ModeloDinamica.cpp : Defines the exported functions for the DLL application.
//
#pragma once
#include "ModeloDinamica.h"
#include "MathHelper.h"
#include "log.h"
#define DINAMICA_LOG(level) FILE_LOG(level) << "Dinamica #" << ": "


namespace sauron
{

namespace modeloDinamica
{

	
ModeloDinamica::ModeloDinamica(ArRobot& robot) 
    : medidaOdometro(robot),
      m_localizationManager( NULL )
{
	init();
}


ModeloDinamica::ModeloDinamica(Pose posicaoEstimada, ArRobot& robot) 
    : medidaOdometro(robot), 
      posicaoEstimada(posicaoEstimada),
      m_localizationManager( NULL )
{
	init();
}


ModeloDinamica::~ModeloDinamica()
{
    m_thread.join();
}


void ModeloDinamica::init()
{
	influLinearLinear   = 0.033;
	influLinearAngular  = 0.001;
	influAngularLinear  = 0.0;
	influAngularAngular = 0.035;

    m_thread = boost::thread::thread( &ModeloDinamica::mainLoop, this );
}


pose_t ModeloDinamica::calculaX()
{
	
	return posicaoEstimada.X() + medidaOdometro.getDeltaDistance()*::cos(posicaoEstimada.Theta());
}


pose_t ModeloDinamica::calculaY()
{
	return posicaoEstimada.Y() + medidaOdometro.getDeltaDistance()*::sin(posicaoEstimada.Theta());
}


pose_t ModeloDinamica::calculaTheta()
{
    if(!floating_point::isEqual(medidaOdometro.getDeltaTheta(), 0))
    {
	    DINAMICA_LOG(logDEBUG2) << "Ultimo Theta Estimado: " << posicaoEstimada.Theta() << ". Delta Theta: " << medidaOdometro.getDeltaTheta();
    }
    return sauron::trigonometry::normalizeAngle((posicaoEstimada.Theta() + medidaOdometro.getDeltaTheta()));
}


double ModeloDinamica::getVarianciaLinear()
{
	double dm = medidaOdometro.getDeltaDistance();
	double thetaM = medidaOdometro.getDeltaTheta();
	return (dm*influLinearLinear)*(dm*influLinearLinear) + (thetaM*influAngularLinear)*(thetaM*influAngularLinear);
}


double ModeloDinamica::getVarianciaAngular()
{
	double dm = medidaOdometro.getDeltaDistance();
	double thetaM = medidaOdometro.getDeltaTheta();
	return (thetaM*influAngularAngular)*(thetaM*influAngularAngular) + (dm*influLinearAngular)*(dm*influLinearAngular);
}

			
void ModeloDinamica::atualizaCovariancia(Covariance &dynNoise)
{

    /*
						        (X)T = transposta de X 
     Q(n) = Fobs(n) . Qobs(n) . (Fobs(n))T
	
    */

    double cosTheta = ::cos(posicaoEstimada.Theta());
    double senTheta = ::sin(posicaoEstimada.Theta());
    double varLinear = getVarianciaLinear();
    double varAngular = getVarianciaAngular();

    dynNoise.resize( 3, 3 );
    dynNoise.clear();

    /* primeira linha */
    dynNoise(0,0) = varLinear*cosTheta*cosTheta;
    dynNoise(0,1) = varLinear*cosTheta*senTheta;
    dynNoise(0,2) = 0;

    /* segunda linha */
    dynNoise(1,0) = dynNoise(0,1);
    dynNoise(1,1) = varLinear*senTheta*senTheta;
    dynNoise(1,2) = 0;

    /* terceira linha */
    dynNoise(2,0) = 0;
    dynNoise(2,1) = 0;
    dynNoise(2,2) = varAngular;	
}


void ModeloDinamica::atualizaModel(Model &dynModel)
{
    dynModel.resize( 3, 3 );
    dynModel.clear();

    /* primeira linha */
    dynModel(0, 0) = 1;
    dynModel(0, 1) = 0;
    dynModel(0, 2) =  -medidaOdometro.getDeltaDistance()*::sin(posicaoEstimada.Theta());

    /* segunda linha */
    dynModel(1, 0) = 0;
    dynModel(1, 1) = 1;
    dynModel(1, 2) =  medidaOdometro.getDeltaDistance()*::cos(posicaoEstimada.Theta());

    /* terceira linha */
    dynModel(2, 0) = 0;
    dynModel(2, 1) = 0;
    dynModel(2, 2) = 1;

}


void ModeloDinamica::atualizaFValue(Matrix &fValue)
{
    fValue.resize( 3, 1 );
    fValue.clear();

    /* uma unica coluna */
    fValue(0, 0) = calculaX();
    fValue(1, 0) = calculaY();
    fValue(2, 0) = calculaTheta();

}


void ModeloDinamica::updateModel( Matrix &fValue, Model &dynModel, Covariance &dynNoise )
{
	// atualizaPose
	posicaoEstimada = m_localizationManager->getPose();

	medidaOdometro.atualizaMedida();
	
	// calcula fValue
	atualizaFValue(fValue);
	
	// calcula F
	atualizaModel(dynModel);

	// atualiza Q
	atualizaCovariancia(dynNoise);
}


void ModeloDinamica::mainLoop()
{
    Matrix     fValue( 1, 1 );
    Matrix     dynModel( 1, 1 );
    Covariance dynNoise( 1, 1 );

    boost::xtime sleepTime;
    boost::xtime sleepDeltaTime;

    sleepDeltaTime.nsec = 50000000;

    while ( true )
    {
        if ( m_localizationManager )
        {
            this->updateModel( fValue, dynModel, dynNoise );
            m_localizationManager->predict( fValue, dynModel, dynNoise );
        }
        
        boost::xtime_get( &sleepTime, boost::TIME_UTC );
        sleepTime.nsec += sleepDeltaTime.nsec;
        m_thread.sleep( sleepTime );
    }
}


void ModeloDinamica::setLocalizationManager( ILocalizationManager &locManager )
{
    m_localizationManager = &locManager;
}


}   // namespace modeloDinamica

}   // namespace sauron
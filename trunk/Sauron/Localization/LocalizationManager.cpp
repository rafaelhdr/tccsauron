#include "Aria.h"
#include "LocalizationManager.h"
#include "ExtendedKalmanFilter.h"
#include "Pose.h"
#include "ModeloDinamica/ModeloDinamica.h"
#include "SensorSonar.h"
#include "SensorVision.h"
#include "Sonar/PhysicalSonars.h"

#include <iostream>

namespace sauron
{
    LocalizationManager::LocalizationManager(ArRobot* robot, const Map& map, const std::string &marksFile )
		: mp_robot(robot),  
          m_map(map), 
          mp_dynamic(buildDefaultDynamic()), 
		  mp_ekf(buildDefaultEKF()), 
          mp_sonarDataProvider(buildDefaultSonarDataProvider()),
          m_visionMarksFilename( marksFile )
	{
		buildDefaultSensors();
	}

    LocalizationManager::LocalizationManager(ArRobot* robot, const Map& map, const std::string &marksFile, const Pose& initialPose)
		: mp_robot(robot),  
          m_map(map), 
          mp_dynamic(buildDefaultDynamic(initialPose)), 
		  mp_ekf(buildDefaultEKF()), 
          mp_sonarDataProvider(buildDefaultSonarDataProvider()),
          m_visionMarksFilename( marksFile )
	{
		buildDefaultSensors();
		setInitialPose(initialPose);
	}

	IDynamicModel* LocalizationManager::buildDefaultDynamic()
	{
		return new modeloDinamica::ModeloDinamica(*mp_robot);
	}

	IDynamicModel* LocalizationManager::buildDefaultDynamic(const Pose& initialPose)
	{
		return new modeloDinamica::ModeloDinamica(initialPose, *mp_robot);
	}

	IKalmanFilter* LocalizationManager::buildDefaultEKF()
	{
		return new ExtendedKalmanFilter();
	}

	ISonarDataAsyncProvider* LocalizationManager::buildDefaultSonarDataProvider()
	{
		//return new PhysicalSonars(mp_robot);
		return 0;
	}

	void LocalizationManager::buildDefaultSensors()
	{
		//buildDefaultSonars();
		//buildDefaultVision();
	}

	void LocalizationManager::buildDefaultSonars()
	{
		for(int i = 0; i < 8; i++) {
			m_sensors.push_back(ISensorModelPtr(new SensorSonar(i, *this, *mp_sonarDataProvider)));
		}
	}

	void LocalizationManager::buildDefaultVision()
	{
        m_sensors.push_back( ISensorModelPtr( new SensorVision( m_visionMarksFilename ) ) );
	}

	void LocalizationManager::startAsync()
	{
		if(m_localizationThread.get_id() == boost::thread::id()) { // true se a thread não foi iniciada
			localize = true;
			m_localizationThread = boost::thread(&LocalizationManager::mainLoop, this);
		}
	}

	void LocalizationManager::stopAsync()
	{
		localize = false;
	}

	void LocalizationManager::mainLoop()
	{

		/* TODO:
			conversando com o Silva achamos q isso precisa ser reestruturado...
			do jeito atual, vc eh obrigado a pegar uma atualização de cada sensor

			o ideal eh cada sensor dar a sua atualização na hora em q ele a tiver... 

			ou então o predict e os updates são disparados em threads distintas... cada uma processando no seu ritmo

		*/

		while(localize) {
			// predict
			Matrix fValue(3,1); Model dynModel(3,3);	Covariance dynNoise(3,3);
			mp_dynamic->updateModel(this->getPose(), fValue, dynModel, dynNoise);
			{
				/* fiz essa alteração pq o setInitial Pose não estava funcionando.... agora eu dou lock e unlock
				e o setInitialPose faz a mesma coisa, assim, se um estiver alterando, o outro vai esperar */

				boost::unique_lock<boost::mutex> lock(m_ekfMutex);
				mp_ekf->predict(fValue, dynModel, dynNoise);
			}

			// update
			for(std::vector<ISensorModelPtr>::iterator it = m_sensors.begin();
				it != m_sensors.end(); it++) {
					Matrix hValue(1,3); Measure z(1,1); Model H(1,3); Covariance R(3,3);
					if((*it)->getEstimate(this->getPose(), hValue, z, H, R)) {
						boost::unique_lock<boost::mutex> lock(m_ekfMutex);
						mp_ekf->update(z, hValue, H, R);
					}
			}
			
		}
	}

	Pose LocalizationManager::getPose()
	{
		boost::unique_lock<boost::mutex>(m_ekfMutex);
		return mp_ekf->getLatestEstimate();
	}

	LocalizationManager::~LocalizationManager(void)
	{
	}
}
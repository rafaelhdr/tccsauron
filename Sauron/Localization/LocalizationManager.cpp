#include "Aria.h"
#include "LocalizationManager.h"
#include "ExtendedKalmanFilter.h"
#include "Pose.h"
#include "ModeloDinamica/ModeloDinamica.h"
#include "SensorSonar.h"
#include "Sonar/PhysicalSonars.h"

#include <iostream>

namespace sauron
{
	LocalizationManager::LocalizationManager(ArRobot* robot, const Map& map)
		: mp_robot(robot),  m_map(map), mp_dynamic(buildDefaultDynamic()), 
		mp_ekf(buildDefaultEKF()), mp_sonarDataProvider(buildDefaultSonarDataProvider())
	{
		buildDefaultSensors();
	}

	LocalizationManager::LocalizationManager(ArRobot* robot, const Map& map,
		const Pose& initialPose)
		: mp_robot(robot),  m_map(map), mp_dynamic(buildDefaultDynamic()), 
		mp_ekf(buildDefaultEKF()), mp_sonarDataProvider(buildDefaultSonarDataProvider())
	{
		buildDefaultSensors();
		setInitialPose(initialPose);
	}

	IDynamicModel* LocalizationManager::buildDefaultDynamic()
	{
		return new modeloDinamica::ModeloDinamica(*mp_robot);
	}

	IKalmanFilter* LocalizationManager::buildDefaultEKF()
	{
		return new ExtendedKalmanFilter();
	}

	ISonarDataAsyncProvider* LocalizationManager::buildDefaultSonarDataProvider()
	{
		return new PhysicalSonars(mp_robot);
	}

	void LocalizationManager::buildDefaultSensors()
	{
		buildDefaultSonars();
		buildDefaultVision();
	}

	void LocalizationManager::buildDefaultSonars()
	{
		for(int i = 0; i < 8; i++) {
			m_sensors.push_back(ISensorModelPtr(new SensorSonar(i, *this, *mp_sonarDataProvider)));
		}
	}

	void LocalizationManager::buildDefaultVision()
	{
		// TODO
	}

	void LocalizationManager::mainLoop()
	{
		while(localize) {
			// predict
			Matrix fValue; Model dynModel;	Covariance dynNoise;
			mp_dynamic->updateModel(this->getPose(), fValue, dynModel, dynNoise);
			mp_ekf->predict(fValue, dynModel, dynNoise);
			// update
			/*
			for(std::vector<ISensorModelPtr>::iterator it = m_sensors.begin();
				it != m_sensors.end(); it++) {
					Matrix hValue; Measure z; Model H; Covariance R;
					if((*it)->getEstimate(this->getPose(), hValue, z, H, R)) {
						mp_ekf->update(z, hValue, H, R);
					}
			}
			*/
			Pose pose = getPose();
			std::cout << "Predição: (" << pose.X() << ", " <<
				pose.Y() << ", " << pose.Theta() << ")" << std::endl;
			::Sleep(1000);
		}
	}

	LocalizationManager::~LocalizationManager(void)
	{
	}
}
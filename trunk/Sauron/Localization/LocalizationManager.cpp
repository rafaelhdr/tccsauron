#include "Aria.h"
#include "LocalizationManager.h"
#include "ExtendedKalmanFilter.h"
#include "Pose.h"
#include "ModeloDinamica/ModeloDinamica.h"
#include "SensorSonar.h"
#include "SensorVision.h"
#include "Sonar/PhysicalSonars.h"
#include <boost/bind.hpp>

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
		addPoseChangedCallback(boost::bind(&LocalizationManager::updateArRobotPose, this, _1));
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
		addPoseChangedCallback(boost::bind(&LocalizationManager::updateArRobotPose, this, _1));
	}


	IDynamicModel* LocalizationManager::buildDefaultDynamic()
	{
		IDynamicModel* dynamic = new modeloDinamica::ModeloDinamica(*mp_robot);
		dynamic->setLocalizationManager(*this);
        return dynamic;
	}

	IDynamicModel* LocalizationManager::buildDefaultDynamic(const Pose& initialPose)
	{
		IDynamicModel* dynamic = new modeloDinamica::ModeloDinamica(initialPose, *mp_robot);
		dynamic->setLocalizationManager(*this);
        return dynamic;
	}

	IKalmanFilter* LocalizationManager::buildDefaultEKF()
	{
		return new ExtendedKalmanFilter();
	}

	ISonarDataAsyncProvider* LocalizationManager::buildDefaultSonarDataProvider()
	{
		return new PhysicalSonars(mp_robot);
		//return 0;
	}

	void LocalizationManager::buildDefaultSensors()
	{
		buildDefaultSonars();
		//buildDefaultVision();
	}

	void LocalizationManager::buildDefaultSonars()
	{
		//m_sensors.push_back(ISensorModelPtr(new SensorSonar(0, *this, *mp_sonarDataProvider)));
		//for(int i = 0; i < 8; i++) {
		for(int i = 0; i < 8; i++) {
			ISensorSonarModelPtr sonarModel = ISensorSonarModelPtr(new SensorSonar(i, *mp_sonarDataProvider));
			sonarModel->setLocalizationManager(*this);
			m_sonars.push_back(sonarModel);
		}
	}

	void LocalizationManager::buildDefaultVision()
	{
        //m_sensors.push_back( ISensorModelPtr( new SensorVision( m_visionMarksFilename ) ) );
	}

	void LocalizationManager::invokePoseChangedCallbacks() {
		invokeCallbacks(this->getPose());
	}

	void LocalizationManager::updateArRobotPose(const Pose& currentPose) {
		mp_robot->moveTo(ArPose(currentPose.X() * 10, currentPose.Y() * 10,
			sauron::trigonometry::rads2degrees(currentPose.Theta())), false);
	}

	void LocalizationManager::update(const Matrix &hValue,
									 const Measure &z,
									 const Model &H,
									 const Covariance &R)
	{
		{
			boost::unique_lock<boost::recursive_mutex> lock(m_ekfMutex);
			mp_ekf->update(z, hValue, H, R);
		}
		invokePoseChangedCallbacks();
	}

	void LocalizationManager::predict(const Matrix &fValue,
									  const Model &dynModel,
									  const Covariance &dynNoise)
	{
		{
			boost::unique_lock<boost::recursive_mutex> lock(m_ekfMutex);
			mp_ekf->predict(fValue, dynModel, dynNoise);
		}
		invokePoseChangedCallbacks();
	}

	Pose LocalizationManager::getPose()
	{
		boost::unique_lock<boost::recursive_mutex> lock(m_ekfMutex);
		return mp_ekf->getLatestEstimate();
	}

	std::vector<SonarMatch> LocalizationManager::getSonarMatches()
	{
		std::vector<SonarMatch> matches;
		for(std::vector<ISensorSonarModelPtr>::const_iterator it = m_sonars.begin(); it != m_sonars.end(); it++)
		{
			if((*it)->hasMatch()) {
				matches.push_back((*it)->getLatestMatch());
			}
		}
		return matches;
	}

	LocalizationManager::~LocalizationManager(void)
	{
	}
}
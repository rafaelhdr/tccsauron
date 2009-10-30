#include "LocalizationManager.h"
#include "ExtendedKalmanFilter.h"
#include "OdometerBasedDynamic.h"
#include "SensorSonar.h"
#include "Sonar/SonarModel.h"
#include "Sonar/Configs.h"

namespace sauron
{
	LocalizationManager::LocalizationManager(const Map& map)
		: m_map(map), mp_dynamic(buildDefaultDynamic()), mp_ekf(buildDefaultEKF())
	{
		buildDefaultSensors();
	}

	IDynamicModel* LocalizationManager::buildDefaultDynamic()
	{
		return new OdometerBasedDynamic();
	}

	IKalmanFilter* LocalizationManager::buildDefaultEKF()
	{
		return new ExtendedKalmanFilter();
	}

	void LocalizationManager::buildDefaultSensors()
	{
		buildDefaultSonars();
		buildDefaultVision();
	}

	void LocalizationManager::buildDefaultSonars()
	{
		for(int i = 0; i < 8; i++) {
			m_sonarModels.push_back(ISonarModelPtr(new SonarModel(configs::sonars::getSonarPose(i))));
		}
	}

	LocalizationManager::~LocalizationManager(void)
	{
	}
}
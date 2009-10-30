#pragma once
#include <vector>
#include <boost/shared_ptr.hpp>

#include "ILocalizationManager.h"
#include "ISensorModel.h"
#include "IDynamicModel.h"
#include "IKalmanFilter.h"
#include "Sonar/ISonarModel.h"
#include "Sonar/Map.h"

namespace sauron
{

typedef boost::shared_ptr<ISensorModel> ISensorModelPtr;
typedef boost::shared_ptr<IDynamicModel> IDynamicModelPtr;
typedef boost::shared_ptr<IKalmanFilter> IKalmanFilterPtr;
// HACK não gosto disso, mas agora não vou mudar (Pedro)
typedef boost::shared_ptr<ISonarModel> ISonarModelPtr;

class LocalizationManager : public ILocalizationManager
{
public:
	LocalizationManager(const Map& map);
	LocalizationManager(const Map& map, const Pose& initialPose);
	~LocalizationManager();

	void setInitialPose(const Pose& initial);
	void startAsync();
	void stopAsync();
	Pose getPose();
	Map getMap() { return m_map; }

private:
	Map m_map;
	IKalmanFilterPtr mp_ekf;
	std::vector<ISensorModelPtr> m_sensors;
	// HACK
	std::vector<ISonarModelPtr> m_sonarModels;
	IDynamicModelPtr mp_dynamic;

	void buildDefaultSensors();
	void buildDefaultSonars();
	void buildDefaultVision();
	IDynamicModel* buildDefaultDynamic();
	IKalmanFilter* buildDefaultEKF();

};
}
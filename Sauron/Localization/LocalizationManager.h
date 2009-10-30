#pragma once
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "ILocalizationManager.h"
#include "ISensorModel.h"
#include "IDynamicModel.h"
#include "IKalmanFilter.h"
#include "Sonar/ISonarDataAsyncProvider.h"
#include "Sonar/Map.h"

class ArRobot;

namespace sauron
{

typedef boost::shared_ptr<ISensorModel> ISensorModelPtr;
typedef boost::shared_ptr<IDynamicModel> IDynamicModelPtr;
typedef boost::shared_ptr<IKalmanFilter> IKalmanFilterPtr;
typedef boost::shared_ptr<ISonarDataAsyncProvider> ISonarDataProviderPtr;

class LocalizationManager : public ILocalizationManager
{
public:
    LocalizationManager(ArRobot* p_robot, const Map& map, const std::string &marksFile );
    LocalizationManager(ArRobot* p_robot, const Map& map, const std::string &markFile, const Pose& initialPose);
	~LocalizationManager();

	void setInitialPose(const Pose& initial){ mp_ekf->setLatestEstimate(initial); }
	void startAsync(){ }
	void stopAsync() { }
	Pose getPose() { return mp_ekf->getLatestEstimate(); }
	Map getMap() { return m_map; }
	void mainLoop();
private:
	ArRobot* mp_robot;
	Map m_map;
	IKalmanFilterPtr mp_ekf;
	std::vector<ISensorModelPtr> m_sensors;
	ISonarDataProviderPtr mp_sonarDataProvider;
	IDynamicModelPtr mp_dynamic;

    std::string m_visionMarksFilename;

	void buildDefaultSensors();
	void buildDefaultSonars();
	void buildDefaultVision();
	
	bool localize;

	ISonarDataAsyncProvider* buildDefaultSonarDataProvider();
	IDynamicModel* buildDefaultDynamic();
	IDynamicModel* buildDefaultDynamic(const Pose&);
	IKalmanFilter* buildDefaultEKF();

};
}
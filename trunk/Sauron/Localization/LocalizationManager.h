#pragma once
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

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

	void setInitialPose(const Pose& initial){
		boost::unique_lock<boost::mutex> lock(m_ekfMutex);
		mp_ekf->setLatestEstimate(initial);
	}

	void addPoseChangedCallback(boost::function<void (const Pose&)> callback);

	Pose getPose();
	Map getMap() { return m_map; }

	void update(const Matrix &hValue,
				const Measure &z,
				const Model &H,
				const Covariance &R);

	void predict(const Matrix &fValue,
				 const Model &dynModel,
				 const Covariance &dynNoise);

private:
	LocalizationManager(LocalizationManager& original);
	ArRobot* mp_robot;
	Map m_map;
	IKalmanFilterPtr mp_ekf;
	std::vector<ISensorModelPtr> m_sensors;
	ISonarDataProviderPtr mp_sonarDataProvider;
	IDynamicModelPtr mp_dynamic;

	std::vector<boost::function<void (const Pose&)> > m_poseChangedCallbacks;
	void invokePoseChangedCallbacks();

    std::string m_visionMarksFilename;

	boost::mutex m_ekfMutex;

	void buildDefaultSensors();
	void buildDefaultSonars();
	void buildDefaultVision();

	ISonarDataAsyncProvider* buildDefaultSonarDataProvider();
	IDynamicModel* buildDefaultDynamic();
	IDynamicModel* buildDefaultDynamic(const Pose&);
	IKalmanFilter* buildDefaultEKF();

};
}
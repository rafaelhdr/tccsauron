#pragma once
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include "ILocalizationManager.h"
#include "CallbackProvider.h"
#include "ISensorModel.h"
#include "ISensorSonarModel.h"
#include "IDynamicModel.h"
#include "IKalmanFilter.h"
#include "Sonar/ISonarDataAsyncProvider.h"
#include "Map.h"
#include "SonarMatch.h"
#include "MapManager.h"

class ArRobot;
namespace sauron
{

class SonarMatch;

typedef boost::shared_ptr<ISensorModel> ISensorModelPtr;
typedef boost::shared_ptr<ISensorSonarModel> ISensorSonarModelPtr;
typedef boost::shared_ptr<IDynamicModel> IDynamicModelPtr;
typedef boost::shared_ptr<IKalmanFilter> IKalmanFilterPtr;
typedef boost::shared_ptr<ISonarDataAsyncProvider> ISonarDataProviderPtr;

class LocalizationManager : public ILocalizationManager, CallbackProvider<boost::function<void (const Pose&)> >
{
public:
    LocalizationManager(ArRobot* p_robot, MapManager& mapManager, const std::string &marksFile );
    LocalizationManager(ArRobot* p_robot, MapManager& mapManager, const std::string &markFile, const Pose& initialPose);
	~LocalizationManager();

	void setInitialPose(const Pose& initial){
		{
			boost::unique_lock<boost::recursive_mutex> lock(m_ekfMutex);
			mp_ekf->setLatestEstimate(initial);
		}
		invokePoseChangedCallbacks();
	}

	int addPoseChangedCallback(boost::function<void (const Pose&)> callback) {
		return addCallback(callback);
	}

	void removePoseChangedCallback(int callbackId) {
		return removeCallback(callbackId);
	}

	ISonarDataAsyncProvider* getSonarDataProvider();

	Pose getPose();
	boost::recursive_mutex* getPoseMutex() { return &m_ekfMutex; }
	Map* getMap() { return m_mapManager.getCurrentMap(); }
	std::vector<SonarMatch> getSonarMatches();

	void update(const Matrix &hValue,
				const Measure &z,
				const Model &H,
				const Covariance &R);

	void predict(const Matrix &fValue,
				 const Model &dynModel,
				 const Covariance &dynNoise);

	Covariance getPoseEstimateCovariance() {
		return mp_ekf->getLatestCovariance();
	}

	void setIsTurning(bool isTurning) {
		m_isTurning = isTurning;
	}

	void freeze() { m_freeze = true; }
	void unfreeze() { m_freeze = false; }

private:
	LocalizationManager(LocalizationManager& original);
	MapManager& m_mapManager;
	ArRobot* mp_robot;
	IKalmanFilterPtr mp_ekf;
	std::vector<ISensorSonarModelPtr> m_sonars;
	std::vector<ISensorModelPtr> m_simpleSonars;
    ISensorModelPtr m_visionSensor;
	ISonarDataProviderPtr mp_sonarDataProvider;
	IDynamicModelPtr mp_dynamic;
	bool m_isTurning;
	bool m_freeze;

	void invokePoseChangedCallbacks();
	void updateArRobotPose(const Pose& newPose);

	void newSonarAssociation(SonarMatch match);

    std::string m_visionMarksFilename;

	boost::recursive_mutex m_ekfMutex;

	void buildDefaultSensors();
	void buildDefaultSonars();
	void buildDefaultSimpleSonars();
	void buildDefaultVision();

	ISonarDataAsyncProvider* buildDefaultSonarDataProvider();
	IDynamicModel* buildDefaultDynamic();
	IDynamicModel* buildDefaultDynamic(const Pose&);
	IKalmanFilter* buildDefaultEKF();

};
}

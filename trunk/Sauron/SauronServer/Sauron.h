#pragma once
#include "stdafx.h"
#include "Localization/LocalizationManager.h"
#include "Localization/MapManager.h"
#include "Navigation/MapPlanner.h"

namespace sauron
{
class Sauron
{
public:
	Sauron(ArRobot* robot, const std::vector<std::string>& maps, const std::string& initialMap,
		const std::string& marksFile = "");
	~Sauron(void);

	// comandos
	bool setInitialMap(const std::string& initialMap);
	bool goTo(const std::string& goalName);
	void setPose(const Pose& pose);
	bool setPose(const std::string& nodeName, pose_t theta);
	void halt();
	void freeze();
	void continueAfterStop();

	// componentes principais
	// localização
	LocalizationManager* getLocalizationManager();
	// navegação
	MapPlanner* getMapPlanner();
private:
	ArRobot* mp_robot;
	std::string m_goal;
	bool m_isMoving;

	boost::shared_ptr<LocalizationManager>  mp_localization;
	boost::shared_ptr<MapPlanner> mp_planner;
	boost::shared_ptr<MapManager> mp_mapManager;
};
}

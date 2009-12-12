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
	Pose getPose();
	void setPose(const Pose& pose);
	bool setPose(const std::string& nodeName, pose_t theta);
	void halt();
	void freeze();
	void continueAfterStop();

	// componentes principais
	// localização
	LocalizationManager* getLocalizationManager() {
		return mp_localization;
	}
	// navegação
	MapPlanner* getMapPlanner() {
		return mp_planner;
	}
private:
	Sauron(const Sauron& other);
	ArRobot* mp_robot;
	std::string m_goal;
	bool m_isMoving;
	
	MapManager* mp_mapManager;
	MapPlanner* mp_planner;
	LocalizationManager*  mp_localization;

	void unfreeze();
};
}

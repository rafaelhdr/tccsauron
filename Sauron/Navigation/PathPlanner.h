#pragma once
#include "Node.h"
#include <string>
class ArRobot;
namespace sauron 
{
	class LocalizationManager;
	class Map;

	class PathPlanner
	{
	public:
		PathPlanner(ArRobot* robot, LocalizationManager* locManager, const std::string& mapFilename);
		bool goTo(const Node& destination);

	private:
		ArRobot* mp_robot;
		LocalizationManager* mp_localization;
		Graph m_graph;

		void loadWaypointsGraphFromMap(const std::string& mapFilename);
		Node getCurrentPoseAsNode();
	};
}

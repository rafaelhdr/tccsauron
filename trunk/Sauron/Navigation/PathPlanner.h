#pragma once
#include "Node.h"
#include "AStar.h"
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
		bool goTo(const std::string& goalName);

		Graph getGraph() { return m_graph; }

	private:
		ArRobot* mp_robot;
		LocalizationManager* mp_localization;
		Graph m_graph;

		void removeNodesTooCloseFromPath(const Node& currentNode, Path &path);
		void loadWaypointsGraphFromMap(const std::string& mapFilename);
		Node getCurrentPoseAsNode();
		bool hasReachedDestination(const Node& destination);
	};
}

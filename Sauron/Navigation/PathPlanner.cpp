#include "PathPlanner.h"
#include "MapFileParser.h"
#include "WaypointLinker.h"
#include "Localization/LocalizationManager.h"

namespace sauron
{
	PathPlanner::PathPlanner(ArRobot* robot, LocalizationManager* locManager, const std::string& mapFilename)
		: mp_robot(robot), mp_localization(locManager) { }

	bool PathPlanner::goTo(const sauron::Node& destination)
	{
		return false;
	}

	void PathPlanner::loadWaypointsGraphFromMap(const std::string& mapFilename)
	{
		util::MapFileParser::loadWaypoints( mapFilename, m_graph );
		util::WaypointLinker::link(m_graph);
	}

	Node PathPlanner::getCurrentPoseAsNode()
	{
		return Node(mp_localization->getPose(), Node::PRIMARY);
	}
}

#pragma once
#include "ConsoleLogger.h"
namespace sauron
{
class PathPlanner;
class MapPlanner;
class Node;
class Map;
enum PathPlannerStatus;
enum MapPlannerStatus;
namespace tests
{

class NavigationMonitorConsole
{
public:
	NavigationMonitorConsole(PathPlanner* planner);
	NavigationMonitorConsole(PathPlanner* planner, MapPlanner* mapPlanner);
	~NavigationMonitorConsole(void);

private:
	CConsoleLogger m_console;
	PathPlanner* mp_planner;
	MapPlanner* mp_mapPlanner;
	int m_callbackId;
	int m_mapPlannerCallbackId;

	void mapPlannerCallback(MapPlannerStatus status, const Map* p_map);
	void callback(PathPlannerStatus status, const Node* p_node);
};
}
}

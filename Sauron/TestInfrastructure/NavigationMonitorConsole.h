#pragma once
#include "ConsoleLogger.h"
namespace sauron
{
class PathPlanner;
class Node;
enum PathPlannerStatus;
namespace tests
{

class NavigationMonitorConsole
{
public:
	NavigationMonitorConsole(PathPlanner* planner);
	~NavigationMonitorConsole(void);

private:
	CConsoleLogger m_console;
	PathPlanner* mp_planner;
	int m_callbackId;

	void callback(PathPlannerStatus status, const Node* p_node);
};
}
}

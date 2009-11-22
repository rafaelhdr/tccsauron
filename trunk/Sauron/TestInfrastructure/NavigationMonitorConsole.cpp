#include "StdAfx.h"
#include "NavigationMonitorConsole.h"
#include "Navigation/PathPlanner.h"

namespace sauron
{
namespace tests
{

NavigationMonitorConsole::NavigationMonitorConsole(PathPlanner* planner)
: mp_planner(planner)
{
	m_console.Create("NavigationMonitor");
	m_callbackId = mp_planner->addPathplanningCallback(
		boost::bind(&NavigationMonitorConsole::callback, this, _1, _2));
}

NavigationMonitorConsole::~NavigationMonitorConsole(void)
{
	mp_planner->removePathplanningCallback(m_callbackId);
}

void NavigationMonitorConsole::callback(sauron::PathPlannerStatus status, const sauron::Node *p_node)
{
	switch(status)
	{
	case GOING_TO_WAYPOINT:
		m_console.printf("Indo para %s\n", p_node->getName().c_str());
		break;
	case ARRIVED_AT_WAYPOINT:
		m_console.printf("Chegou em %s\n", p_node->getName().c_str());
		break;
	case GOAL_REACHED:
		m_console.printf("Chegou no objetivo: %s\n", p_node->getName().c_str());
		break;
	case FAILED_COLLISION_AVOIDANCE:
		m_console.printf("ERRO: Parou para evitar colisao\n");
		break;
	case FAILED_STRAYED:
		m_console.printf("ERRO: Desviou-se da rota\n");
		break;
	default:
		m_console.printf("WARNING: Codigo de callback desconhecido: %d\n", status);
		break;
	}
}

}
}
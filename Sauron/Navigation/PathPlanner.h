#pragma once
#include "Node.h"
#include "AStar.h"
#include <string>
#include <boost/function.hpp>
#include "CallbackHandler.h"

class ArRobot;
namespace sauron 
{
	class LocalizationManager;
	class Map;
	class PathPlanner;
	enum PathPlannerStatus
		{
			GOING_TO_WAYPOINT, // Node* = próximo waypoint/goal
			ARRIVED_AT_WAYPOINT, // Node* = waypoint alcançado
			FAILED_STRAYED, // Node* = null
			FAILED_COLLISION_AVOIDANCE, // Node* = null
			GOAL_REACHED // Node = goal
		};

	class PathPlanner : CallbackHandler<boost::function<void (PathPlannerStatus, const Node*)> >
	{
	public:
		PathPlanner(ArRobot* robot, LocalizationManager* locManager, const std::string& mapFilename);
		bool goTo(const Node& destination);
		bool goTo(const std::string& goalName);

		Graph getGraph() { return m_graph; }

		int addPathplanningCallback(boost::function<void (PathPlannerStatus, const Node*)> f)
		{
			return addCallback(f);
		}

		void  removePathplanningCallback(int callbackId)
		{
			removeCallback(callbackId);
		}

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

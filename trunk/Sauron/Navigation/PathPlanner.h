#pragma once
#include "Node.h"
#include "AStar.h"
#include <string>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include "CallbackProvider.h"

class ArRobot;
namespace sauron 
{
	class LocalizationManager;
	class Map;
	class PathPlanner;
	class RouteExecuter;
	enum PathPlannerStatus
		{
			GOING_TO_WAYPOINT, // Node* = próximo waypoint/goal
			ARRIVED_AT_WAYPOINT, // Node* = waypoint alcançado
			FAILED_STRAYED, // Node* = null
			FAILED_COLLISION_AVOIDANCE, // Node* = null
			GOAL_REACHED // Node = goal
		};

	class PathPlanner : CallbackProvider<boost::function<void (PathPlannerStatus, const Node*)> >
	{
	public:
		PathPlanner(ArRobot* robot, LocalizationManager* locManager);
		bool goTo(const Node& destination, Graph& graph);
		bool goTo(const std::string& goalName, Graph& graph);

		int addPathplanningCallback(boost::function<void (PathPlannerStatus, const Node*)> f)
		{
			return addCallback(f);
		}

		void  removePathplanningCallback(int callbackId)
		{
			removeCallback(callbackId);
		}

		bool halt();

	private:
		ArRobot* mp_robot;
		LocalizationManager* mp_localization;

		RouteExecuter* mp_executer;
		boost::mutex m_routeExecuterMutex;
		void removeNodesTooCloseFromPath(const Node& currentNode, Path &path);
		//void loadWaypointsGraphFromMap(const std::string& mapFilename);
		Node getCurrentPoseAsNode();
		bool hasReachedDestination(const Node& destination);
	};
}

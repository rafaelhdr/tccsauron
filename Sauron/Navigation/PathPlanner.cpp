#include "Aria.h"
#include "PathPlanner.h"
#include "WaypointLinker.h"
#include "RouteExecuter.h"
#include "RobotController.h"
#include "Localization/LocalizationManager.h"
#include "Map.h"

namespace sauron
{
	PathPlanner::PathPlanner(ArRobot* robot, LocalizationManager* locManager)
		: mp_robot(robot), mp_localization(locManager)
	{
		robotController::setRobot(robot);
		//loadWaypointsGraphFromMap(mapFilename);
	}

	bool PathPlanner::goTo(const std::string& name,  Graph& graph)
	{
		for(Graph::iterator it = graph.begin(); it != graph.end(); it++) {
			if(it->getName() == name) {
				return goTo(*it, graph);
			}
		}

		// ops, não achou esse nome
		return false;
	}

	bool PathPlanner::goTo(const sauron::Node& destination, Graph& graph)
	{
		if(!hasReachedDestination(destination))
		{
			Node currentNode = getCurrentPoseAsNode();
			util::WaypointLinker::linkTemporaryNode(graph, currentNode, destination, mp_localization->getMap());
			Path path = AStar::searchPath(currentNode, destination);
			removeNodesTooCloseFromPath(currentNode, path);
			
			if(path.size() > 0) {
				invokeCallbacks(GOING_TO_WAYPOINT, &path[0]);
				RouteExecuter::MoveResult moveResult = RouteExecuter(mp_robot, mp_localization).goTo(path[0].getPosition());
				if(moveResult == RouteExecuter::SUCCESS) {
					invokeCallbacks(ARRIVED_AT_WAYPOINT, &path[0]);
					return goTo(destination, graph);
				} else if(moveResult == RouteExecuter::FAILED_STRAYED) {
					// desviou-se da rota. há algo de podre no reino da dinamarca, mas somos brasileiros e não desistimos nunca.
					invokeCallbacks(FAILED_STRAYED, &Node());
					return goTo(destination, graph);
				} else if(moveResult == RouteExecuter::FAILED_EMERGENCY_STOP) {
					// algum idiota pôs o pé na frente, ou estamos perdidinhos. vamos tentar de novo, mas com parcimônia
					// TODO dormir por alguns segundos antes de tentar de novo
					// TODO contar quantas vezes falhamos e desistir depois de um tempo
					invokeCallbacks(FAILED_COLLISION_AVOIDANCE,  &Node());
					return goTo(destination, graph);
				}
			}
		}
		else {
			invokeCallbacks(GOAL_REACHED, &destination);
			return true;
		}
		return false; // caminho não foi encontrado, ou não conseguiu se movimentar
	}

	void PathPlanner::removeNodesTooCloseFromPath(const Node& currentNode, Path &path)
	{
		Path cleanPath;
		const int minimalDistanceBetweenNodes = 40; // cm
		for(Path::iterator it = path.begin(); it != path.end(); it++)
		{
			if(it->getPosition().getDistance(currentNode.getPosition()) > minimalDistanceBetweenNodes)
				cleanPath.push_back(*it);
		}
		path.clear();
		path.insert(path.begin(), cleanPath.begin(), cleanPath.end());
	}

	//void PathPlanner::loadWaypointsGraphFromMap(const std::string& mapFilename)
	//{
	//	ArMap map;
	//	map.readFile(mapFilename.c_str());
	//	util::MapFileParser::loadWaypoints( mapFilename, m_graph );
	//	util::WaypointLinker::link(m_graph, Map(map));
	//}

	Node PathPlanner::getCurrentPoseAsNode()
	{
		return Node(mp_localization->getPose(), Node::TEMPORARY, "CurrentPose");
	}

	bool PathPlanner::hasReachedDestination(const Node& destination)
	{
		const int okDistance = 30; // cm
		return mp_localization->getPose().getDistance(destination.getPosition()) < okDistance;
	}
}

#include "PathPlanner.h"
#include "MapFileParser.h"
#include "WaypointLinker.h"
#include "RouteExecuter.h"
#include "RobotController.h"
#include "Localization/LocalizationManager.h"

namespace sauron
{
	PathPlanner::PathPlanner(ArRobot* robot, LocalizationManager* locManager, const std::string& mapFilename)
		: mp_robot(robot), mp_localization(locManager)
	{
		robotController::setRobot(robot);
		loadWaypointsGraphFromMap(mapFilename);
	}

	bool PathPlanner::goTo(const std::string& name)
	{
		for(Graph::iterator it = m_graph.begin(); it != m_graph.end(); it++) {
			if(it->getName() == name) {
				return goTo(*it);
			}
		}

		// ops, não achou esse nome
		return false;
	}

	bool PathPlanner::goTo(const sauron::Node& destination)
	{
		if(!hasReachedDestination(destination))
		{
			Node currentNode = getCurrentPoseAsNode();
			util::WaypointLinker::linkNodeToNearest(m_graph, currentNode);
			Path path = AStar::searchPath(currentNode, destination);
			removeNodesTooCloseFromPath(currentNode, path);

			if(path.size() > 0) {
				RouteExecuter::MoveResult moveResult = RouteExecuter(mp_robot, mp_localization).goTo(path[0].getPosition());
				if(moveResult == RouteExecuter::SUCCESS) {
					return goTo(destination);
				} else if(moveResult == RouteExecuter::FAILED_STRAYED) {
					// desviou-se da rota. há algo de podre no reino da dinamarca, mas somos brasileiros e não desistimos nunca.
					return goTo(destination);
				} else if(moveResult == RouteExecuter::FAILED_EMERGENCY_STOP) {
					// algum idiota pôs o pé na frente, ou estamos perdidinhos. vamos tentar de novo, mas com parcimônia
					// TODO dormir por alguns segundos antes de tentar de novo
					// TODO contar quantas vezes falhamos e desistir depois de um tempo
					return goTo(destination);
				}
			}
		}
		else {
			return true;
		}
		return false; // caminho não foi encontrado, ou não conseguiu se movimentar
	}

	void PathPlanner::removeNodesTooCloseFromPath(const Node& currentNode, Path &path)
	{
		Path cleanPath;
		const int minimalDistanceBetweenNodes = 20; // cm
		for(Path::iterator it = path.begin(); it != path.end(); it++)
		{
			if(it->getPosition().getDistance(currentNode.getPosition()) > minimalDistanceBetweenNodes)
				cleanPath.push_back(*it);
		}
		path.clear();
		path.insert(path.begin(), cleanPath.begin(), cleanPath.end());
	}

	void PathPlanner::loadWaypointsGraphFromMap(const std::string& mapFilename)
	{
		util::MapFileParser::loadWaypoints( mapFilename, m_graph );
		util::WaypointLinker::link(m_graph);
	}

	Node PathPlanner::getCurrentPoseAsNode()
	{
		return Node(mp_localization->getPose(), Node::TEMPORARY);
	}

	bool PathPlanner::hasReachedDestination(const Node& destination)
	{
		const int okDistance = 30; // cm
		return mp_localization->getPose().getDistance(destination.getPosition()) < okDistance;
	}
}

#include "Aria.h"
#include "PathPlanner.h"
#include "WaypointLinker.h"
#include "RouteExecuter.h"
#include "RobotController.h"
#include "Localization/LocalizationManager.h"
#include "Map.h"
#include <boost/thread/thread.hpp>

namespace sauron
{
	PathPlanner::PathPlanner(ArRobot* robot, LocalizationManager* locManager)
		: mp_robot(robot), mp_localization(locManager), mp_executer(0)
	{
		robotController::setRobot(robot);
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
		static int numberCollisionAvoided = 0;
		if(!hasReachedDestination(destination))
		{
			Node currentNode = getCurrentPoseAsNode();
			util::WaypointLinker::linkTemporaryNode(graph, currentNode, destination, mp_localization->getMap());
			Path path = AStar::searchPath(currentNode, destination);
			removeNodesTooCloseFromPath(currentNode, path);
			
			if(path.size() > 0) {
				invokeCallbacks(GOING_TO_WAYPOINT, &path[0]);
				
				{
					boost::unique_lock<boost::mutex> lock(m_routeExecuterMutex);
					mp_executer = new RouteExecuter(mp_robot, mp_localization);
				}

				RouteExecuter::MoveResult moveResult = mp_executer->goTo(path[0].getPosition());
				
				{
					boost::unique_lock<boost::mutex> lock(m_routeExecuterMutex);
					delete mp_executer; mp_executer = 0;
				}

				if(moveResult == RouteExecuter::SUCCESS) {
					numberCollisionAvoided = 0;
					invokeCallbacks(ARRIVED_AT_WAYPOINT, &path[0]);
					return goTo(destination, graph);
				} else if(moveResult == RouteExecuter::FAILED_STRAYED) {
					// desviou-se da rota. há algo de podre no reino da dinamarca, mas somos brasileiros e não desistimos nunca.
					numberCollisionAvoided = 0;
					invokeCallbacks(FAILED_STRAYED, &Node());
					return goTo(destination, graph);
				} else if(moveResult == RouteExecuter::FAILED_EMERGENCY_STOP) {
                    if( ++numberCollisionAvoided >= 5) 
                    {
                        invokeCallbacks(FAILED_OBSTRUCTED_PATH, &Node() );
                        numberCollisionAvoided = 0;
                    }
                    else
                    {
					    // algum idiota pôs o pé na frente, ou estamos perdidinhos. vamos tentar de novo, mas com parcimônia
					    invokeCallbacks(FAILED_COLLISION_AVOIDANCE,  &Node());
					    numberCollisionAvoided++;					
					}
					boost::this_thread::sleep(boost::posix_time::seconds(3)); 

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

	bool PathPlanner::halt()
	{
		boost::unique_lock<boost::mutex> lock(m_routeExecuterMutex);
		if(mp_executer != 0) {
			mp_executer->halt();
			return true;
		} else {
			return false;
		}
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

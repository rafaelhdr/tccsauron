#include "MapPlanner.h"
#include "Aria.h"
#include "Localization/LocalizationManager.h"
#include "Localization/MapManager.h"
#include "MapFileParser.h"
#include "WaypointLinker.h"
#include <vector>
#include <deque>
#include <exception>
namespace sauron
{
	MapPlanner::MapPlanner(ArRobot* robot, sauron::MapManager *mapManager)
		: mp_mapManager(mapManager),
		mp_robot(robot)
	{
		std::vector<std::string> mapNames = mapManager->getMapNames();
		bool initialMapSet = false;
		std::vector<std::string>::iterator it;
		for(it = mapNames.begin(); it != mapNames.end(); it++)
		{
			ArMap arMap;
			if(!arMap.readFile(it->c_str()))
				throw std::exception((std::string("Mapa nao encontrado: ") + *it).c_str());

			m_maps.push_back(new Map(arMap));
			if(mp_mapManager->getInitialMapName() == *it)
			{
				mp_mapManager->setCurrentMap(m_maps.back());
				initialMapSet = true;
			}
			loadWaypointsGraph(m_maps.back());
		}

		if(!initialMapSet)
			throw std::exception(std::string("O mapa inicial nao foi encontrado").c_str());

		findLinksBetweenMaps();
	}


	void MapPlanner::setLocalizationManager(sauron::LocalizationManager *localizationManager)
	{
		mp_localization = localizationManager;
		mp_pathPlanner = new PathPlanner(mp_robot, localizationManager);
	}

	void MapPlanner::findLinksBetweenMaps()
	{
		std::vector<Map*>::iterator mapIt;
		for(mapIt = m_maps.begin(); mapIt != m_maps.end(); mapIt++)
		{
			Graph::iterator nodeIt;
			Graph mapGraph = m_mapGraphs[*mapIt];
			for(nodeIt = mapGraph.begin(); nodeIt != mapGraph.end(); nodeIt++)
			{
				std::vector<Map*>::iterator otherMapIt;
				for(otherMapIt = m_maps.begin(); otherMapIt != m_maps.end(); otherMapIt++)
				{
					if(otherMapIt != mapIt)
					{
						if(isNodeInMap(**otherMapIt,nodeIt->getName()))
						{
							m_mapLinks[*mapIt].push_back(*otherMapIt);
						}
					}
				}
			}
		}
	}

	void MapPlanner::loadWaypointsGraph(Map* map)
	{
		m_mapGraphs[map] = Graph();
		util::MapFileParser::loadWaypoints( map, m_mapGraphs[map] );
		util::WaypointLinker::link(m_mapGraphs[map], map);
		
	}

	bool MapPlanner::goTo(std::string destinationName)
	{
		std::vector<Map*>::iterator mapIt;
		for(mapIt = m_maps.begin(); mapIt != m_maps.end(); mapIt++)
		{
			Graph graph = m_mapGraphs[*mapIt];
			for(Graph::iterator nodeIt = graph.begin(); nodeIt != graph.end(); nodeIt++)
			{
				if(nodeIt->getName() == destinationName)
				{
					return goTo(*nodeIt);
				}
			}
		}

		return false;
	}

	Graph& MapPlanner::getGraph(Map* map)
	{
		return m_mapGraphs[map];

	}

	bool MapPlanner::isNodeInMap(Map& map, std::string nodeName)
	{
		Graph graph = m_mapGraphs[&map];
		for(Graph::iterator nodeIt = graph.begin(); nodeIt != graph.end(); nodeIt++)
		{
			if(nodeIt->getName() == nodeName)
			{
				return true;
			}
		}

		return false;
	}

	Map* MapPlanner::findMapByNode(std::string& nodeName)
	{
		std::vector<Map*>::iterator mapIt;
		for(mapIt = m_maps.begin(); mapIt != m_maps.end(); mapIt++)
		{
			Graph graph = m_mapGraphs[*mapIt];
			for(Graph::iterator nodeIt = graph.begin(); nodeIt != graph.end(); nodeIt++)
			{
				if(nodeIt->getName() == nodeName)
				{
					return *mapIt;
				}
			}
		}
		return 0;
	}

	bool MapPlanner::goTo(Node& destination)
	{
		if(mp_pathPlanner == 0)
			return false;

		Map* destinationMap = findMapByNode(destination.getName());
		if(destinationMap == mp_mapManager->getCurrentMap())
		{
			invokeCallbacks(GOAL_SAME_MAP, destinationMap);
			return mp_pathPlanner->goTo(destination, m_mapGraphs[destinationMap]);
		}
		else
		{
			invokeCallbacks(GOAL_OTHER_MAP, destinationMap);
			std::vector<Map*> mapPath = findMapPath(destinationMap);
			if(mapPath.size() > 0)
			{
				Map* nextMap = mapPath[0];
				Node portalNodeCurrentSide;
				Node portalNodeOtherSide;
				getPortal(*(mp_mapManager->getCurrentMap()), nextMap, &portalNodeCurrentSide,
					&portalNodeOtherSide);
				bool ok = mp_pathPlanner->goTo(portalNodeCurrentSide,
					m_mapGraphs[mp_mapManager->getCurrentMap()]);
				if(ok)
				{
					mp_mapManager->setCurrentMap(nextMap);
					if(mp_localization != 0)
					{
						Point2DDouble newPosition = portalNodeOtherSide.getPosition();
						double oldTheta = mp_localization->getPose().getTheta();
						mp_localization->setInitialPose(
							Pose(newPosition.X(), newPosition.Y(), oldTheta));
					}
					invokeCallbacks(MAP_CHANGED, nextMap);
					return goTo(destination);
				}
				else
					return false;
			}
			else
				return false;
		}
	}



	std::vector<Map*> MapPlanner::findMapPath(sauron::Map *destinationMap)
	{
		std::deque<Map*> queuePath;
		std::vector<Map*> path;
		if(dfs(destinationMap, mp_mapManager->getCurrentMap(), queuePath, std::set<Map*>()))
		{
			while(queuePath.size() != 0)
			{
				path.push_back(queuePath.back());
				queuePath.pop_back();
			}
		}
		path.pop_back(); // retira o mapa atual
		return path;
	}

	bool MapPlanner::dfs(sauron::Map *destinationMap, Map* current, std::deque<Map*>& path, std::set<Map*>& visited)
	{
		
		path.push_back(current);

		if(current == destinationMap)
		{
			return true;
		}

		std::vector<Map*> children = m_mapLinks[current];
		
		if(children.size() == 0)
		{
			path.pop_back();
			return false;
		}
	
		std::vector<Map*>::iterator it;
		for(it = children.begin(); it != children.end(); it++)
		{
			if(visited.find(*it) == visited.end())
			{
				if(dfs(destinationMap, *it, path, visited))
					return true;
			}
		}

		path.pop_back();
		return false;
	}

	void MapPlanner::getPortal(Map& currentMap, Map* nextMap, Node* currentSideNode, Node* otherSideNode)
	{

			Graph currentMapGraph = m_mapGraphs[&currentMap];
			Graph nextMapGraph = m_mapGraphs[&(*nextMap)];

			for(Graph::iterator nodeIt = currentMapGraph.begin(); nodeIt != currentMapGraph.end(); nodeIt++)
			{
				
				for(Graph::iterator nextMapNodeIt = nextMapGraph.begin();
						nextMapNodeIt != nextMapGraph.end();
							nextMapNodeIt++)
				{

					if(nodeIt->getName() == nextMapNodeIt->getName())
					{
						
						*currentSideNode = *nodeIt;
						*otherSideNode = *nextMapNodeIt;
						return;
					}

				}

			}


	}
}
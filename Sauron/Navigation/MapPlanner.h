#pragma once
#include <map>
#include <string>
#include "Node.h"
#include "Map.h"
#include "PathPlanner.h"
class ArRobot;
namespace sauron 
{
	class LocalizationManager;
	class MapManager;

	enum MapPlannerStatus
		{
			GOAL_SAME_MAP, // Map* = currentMap
			GOAL_OTHER_MAP, // Map* = mapa do destino
			MAP_CHANGED // Map* novo mapa
		};

	class MapPlanner : CallbackProvider<boost::function<void (MapPlannerStatus, const Map*)> >
	{
	public:
		MapPlanner(ArRobot* robot, MapManager* mapManager);

		bool goTo(std::string destinationName);
		bool goTo(Node& destinationNode);
		Graph& getGraph(Map* map);

		int addMapPlannerCallback(boost::function<void (MapPlannerStatus, const Map*)> f)
		{
			return addCallback(f);
		}

		void  removeMapPlannerCallback(int callbackId)
		{
			removeCallback(callbackId);
		}

		PathPlanner& getPathPlanner() { return *mp_pathPlanner; }
		void setLocalizationManager(sauron::LocalizationManager *localizationManager);

		~MapPlanner()
		{
			delete mp_pathPlanner;
		}
	private:
		MapManager* mp_mapManager;
		LocalizationManager* mp_localization;
		ArRobot* mp_robot;
		

		std::vector<Map*> m_maps;
		std::map<Map*, Graph> m_mapGraphs;
		typedef std::map<Map*, std::vector<Map*> > MapGraph;
		MapGraph m_mapLinks;
		PathPlanner* mp_pathPlanner;

		bool dfs(sauron::Map *destinationMap, Map* current, std::deque<Map*>& path, std::set<Map*>& visited);
		void getPortal(Map& currentMap, Map* nextMap, Node* currentSideNode, Node* otherSideNode);
		std::vector<Map*> findMapPath(Map* destinationMap);
		Map* findMapByNode(std::string& nodeName);
		bool isNodeInMap(Map& map, std::string nodeName);
		void loadWaypointsGraph(Map* map, std::string mapName);
		void findLinksBetweenMaps();
	};


}

#pragma once
#include <vector>
#include <string>

namespace sauron 
{

	class MapManager
	{
	public:
		MapManager(const std::string& initialMapName, const std::vector<std::string>& mapNames)
			: m_initialMapName(initialMapName), m_mapNames(mapNames)
		{
		}

		Map* getCurrentMap() { return m_currentMap; }

		void setCurrentMap(Map* map) { m_currentMap = map; }

		std::vector<std::string> getMapNames() { return m_mapNames; }
		std::string getInitialMapName() { return m_initialMapName; }

	private:
		Map* m_currentMap;
		std::string m_initialMapName;
		std::vector<std::string> m_mapNames;
	};
}

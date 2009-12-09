#pragma once
#include "Aria.h"
#include <vector>
#include <string>
#include "LineSegment.h"
namespace sauron
{
class Map
{
public:
	Map(ArMapInterface& map)
		: m_originalMapFilename(map.getFileName())
	{
		map.lock();
		std::vector<ArLineSegment>* pLines = map.getLines();
		m_lines.insert(m_lines.begin(), pLines->begin(), pLines->end());
		map.unlock();
	}
	inline std::vector<LineSegment>* getLines()
	{
		return &m_lines;
	}
	inline std::string getOriginalMapFilename() const { return m_originalMapFilename; }
private:
	std::vector<LineSegment> m_lines;
	std::string m_originalMapFilename;
};
}

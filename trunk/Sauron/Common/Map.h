#pragma once
#include "Aria.h"
#include <vector>
#include "LineSegment.h"
namespace sauron
{
class Map
{
public:
	Map(ArMapInterface& map)
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
private:
	std::vector<LineSegment> m_lines;
};
}

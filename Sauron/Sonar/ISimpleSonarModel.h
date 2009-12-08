#pragma once
#include <utility>
#include "LineSegment.h"
namespace sauron
{
	class SonarReading;
	class Map;
	class Pose;

	class ISimpleSonarModel
	{
	public:
		virtual std::pair<double, LineSegment> getExpectedReading(Map* map, const Pose& pose) = 0;
	};
}

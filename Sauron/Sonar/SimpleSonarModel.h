#pragma once
#include "ISimpleSonarModel.h"
#include <utility>
#include "Pose.h"
#include "LineSegment.h"

namespace sauron
{
	class SimpleSonarModel : public ISimpleSonarModel
	{
	public:
		SimpleSonarModel(int sonarNumber);
		std::pair<double, LineSegment> getExpectedReading(Map* map, const Pose& pose);

	private:
		int m_sonarNumber;
		Pose m_relativePose;

		Pose getSonarGlobalPose(const Pose& robotPose);
		ArLineSegment getSonarSegment(const Pose& robotPose);
	};
}

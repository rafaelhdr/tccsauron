#include "SimpleSonarModel.h"
#include "MathHelper.h"
#include "Configs.h"
#include "Map.h"
#include "Pose.h"
#include "ariaUtil.h"
#include <utility>

namespace sauron
{

	SimpleSonarModel::SimpleSonarModel(int sonarNumber)
		: m_sonarNumber(sonarNumber), m_relativePose(configs::sonars::getSonarPose(sonarNumber)){
	}

	Pose SimpleSonarModel::getSonarGlobalPose(const Pose& robotPose) {
		pose_t globalSonarX, globalSonarY, globalSonarTh;
		globalSonarX = robotPose.X() +
			(m_relativePose.X() * ::cos(robotPose.getTheta()) - m_relativePose.Y() * ::sin(robotPose.getTheta()));
		globalSonarY = robotPose.Y() +
			(m_relativePose.X() * ::sin(robotPose.getTheta()) + m_relativePose.Y() * ::cos(robotPose.getTheta()));
		globalSonarTh = robotPose.getTheta() + m_relativePose.Theta();

		return Pose(globalSonarX, globalSonarY, globalSonarTh);
	}

	ArLineSegment SimpleSonarModel::getSonarSegment(const sauron::Pose &sonarPose)
	{
		const int segmentLength = 400;
		double deltaX = segmentLength * ::cos(sonarPose.Theta());
		double deltaY = segmentLength * ::sin(sonarPose.Theta());

		double x1 = sonarPose.X(), y1 = sonarPose.Y();
		double x2 = x1 + deltaX, y2 = y1 + deltaY;
		
		return ArLineSegment(x1, y1, x2, y2);
	}

	std::pair<double, LineSegment> SimpleSonarModel::getExpectedReading(Map* pmap, const Pose& pose)
	{
		if(pmap == 0 || pmap->getLines()->size() == 0)
			return std::pair<double, LineSegment>(-1, LineSegment());

		Pose sonarPose = getSonarGlobalPose(pose);
		ArLineSegment sonarSegment = getSonarSegment(sonarPose);

		std::vector<LineSegment>* mapLines = pmap->getLines();
		typedef std::pair<LineSegment, ArPose> LinePosePair;
		typedef std::vector<LinePosePair> LinePoseVec;
		LinePoseVec linesThatIntersect;
		for(std::vector<LineSegment>::iterator it = mapLines->begin(); it != mapLines->end(); it++)
		{
			ArPose intersection;
			if(sonarSegment.intersects(&(*it), &intersection)) {
				linesThatIntersect.push_back(LinePosePair(*it, intersection));
			}
		}

		LinePoseVec::iterator it;
		LinePoseVec::iterator it_closest;

		double minimumDistance = -1;
		for(it = linesThatIntersect.begin(); it != linesThatIntersect.end(); it++)
		{
			double distance = Pose(it->second.getX(), it->second.getY(), 0).getDistance(sonarPose);
			if(minimumDistance == -1 || minimumDistance > distance)
			{
				minimumDistance = distance;
				it_closest = it;
			}
		}

		if( minimumDistance != -1)
		{
			return std::pair<double, LineSegment>(minimumDistance, it_closest->first);
		} else {
			return std::pair<double, LineSegment>(-1, LineSegment());
		}

		
	}
}
#pragma once
#include <vector>
#include "SonarReading.h"
#include "Pose.h"
#include "Map.h"
#include "LineSegment.h"

#ifdef _CLR_
namespace SonarUnitTests{
ref class SonarTest;
}
#endif

namespace sauron
{
	class Line;

	class SonarModel
	{
	public:
#ifdef _CLR_
		friend ref class SonarUnitTests::SonarTest;
#endif
		SonarModel(const sauron::Pose& sonarPose)
			: m_sonarX(sonarPose.X()),
			m_sonarY(sonarPose.Y()),
			m_sonarTheta(sonarPose.getTheta()) {
		}

		SonarModel(pose_t x, pose_t y, pose_t theta)
			: m_sonarX(x), m_sonarY(y), m_sonarTheta(theta) {
		}

		void addReading(const SonarReading& reading, const Pose& estimatedPose);
		Line getObservedLine();
		bool validateReadings();
		Pose getSonarGlobalPose(const Pose& robotGlobalPose);
		bool tryGetMatchingMapLine(Map& map, /*out */ LineSegment* matchedMapLine,
			/* out */ SonarReading* expectedReading, double sigmaError2);
	private:
		pose_t m_sonarX, m_sonarY, m_sonarTheta;

		std::vector<double> getGammas();
		double getObsMediaVariance();
		double getSinAlpha();
		double getS2_D();
		double getS2_R();
		double getD_Robot();
		double getD_Sonar();
		double getSonarAngleOfIncidence();
		std::vector<LineSegment> filterFarAwayLines(std::vector<LineSegment>& mapLines, const Pose& robotPose);
		std::vector<LineSegment> filterBySonarAngle(std::vector<LineSegment>& mapLines, const Pose& robotPose);
		SonarReading getExpectedReadingByMapLine(const LineSegment& lineSegment);
		bool matchMapLineWithReading(const SonarReading& reading, const LineSegment& mapLine,
			double sigmaError2);
		struct ReadingAndPose {
			ReadingAndPose(const SonarReading& _reading, const Pose& _estimatedPose)
				: reading(_reading), estimatedPose(_estimatedPose) { }
			SonarReading reading;
			Pose estimatedPose;
		};

		bool isReadingMeaningful(const ReadingAndPose& readingAndPose) {
			return readingAndPose.reading.getReading() < configs::sonars::invalidReading;
		}


		std::vector<ReadingAndPose> m_readings;
		ReadingAndPose& getLatestReading() {
			return m_readings.back();
		}
		ReadingAndPose& getOldestReading() {
			return *m_readings.begin();
		}
	};

}
#pragma once
#include <vector>
#include "SonarReading.h"
#include "Pose.h"


namespace SonarUnitTests{
ref class SonarTest;
}

namespace sauron
{
	class Line;

	class Sonar
	{
	public:
		friend ref class SonarUnitTests::SonarTest;
		Sonar(const sauron::Pose& sonarPose)
			: m_sonarX(sonarPose.X()),
			m_sonarY(sonarPose.Y()),
			m_sonarTheta(sonarPose.getTheta()) {
		}

		Sonar(pose_t x, pose_t y, pose_t theta)
			: m_sonarX(x), m_sonarY(y), m_sonarTheta(theta) {
		}

		void addReading(const SonarReading& reading, const Pose& estimatedPose);
		Line getObservedLine();
		bool validateReadings();
		Pose getSonarGlobalPose(const Pose& robotGlobalPose);
	private:
		pose_t m_sonarX, m_sonarY, m_sonarTheta;

		std::vector<double> getGammas();
		double getObsMediaVariance();
		double getSinAlpha();
		double getS2_D();
		double getS2_R();
		double getD_Robot();
		double getD_Sonar();
		struct ReadingAndPose {
			ReadingAndPose(const SonarReading& _reading, const Pose& _estimatedPose)
				: reading(_reading), estimatedPose(_estimatedPose) { }
			SonarReading reading;
			Pose estimatedPose;
		};

		bool isReadingMeaningful(const ReadingAndPose& readingAndPose) {
			if(m_readings.size() == 0) {
				return true;
			} else {
				return getLatestReading().estimatedPose.getDistance(
					readingAndPose.estimatedPose) > configs::sonars::minimalRobotDistance;
			}
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
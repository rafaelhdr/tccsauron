#pragma once
#include <vector>
#include "SonarReading.h"
#include "Pose.h"

namespace sauron
{

	class Sonar
	{
	public:
		Sonar(pose_t x, pose_t y, pose_t theta)
			: m_sonarX(x), m_sonarY(y), m_sonarTheta(theta) {
		}

		void addReading(const SonarReading& reading, const Pose& estimatedPose) {
			ReadingAndPose rnp(reading, estimatedPose);
			m_readings.push_back(rnp);
		}
		bool validateReadings();

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

		std::vector<ReadingAndPose> m_readings;
		ReadingAndPose& getLatestReading() {
			return *m_readings.end();
		}
		ReadingAndPose& getOldestReading() {
			return *m_readings.begin();
		}
	};

}
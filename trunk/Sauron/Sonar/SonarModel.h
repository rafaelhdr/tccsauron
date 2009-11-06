#pragma once
#include <vector>

// A defini��o _CLR_ � ativada quando o projeto est� sendo compilado
// para testes unit�rios. Por algum motivo c�smico, a mera inclus�o do
// header do mutex do Boost faz com que os testes n�o carreguem.
#include <boost/circular_buffer.hpp>
#ifndef _CLR_
# include <boost/thread/recursive_mutex.hpp>
#endif

#include "ISonarModel.h"
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

	class SonarModel : public ISonarModel
	{
	public:
#ifdef _CLR_
		friend ref class SonarUnitTests::SonarTest;
#endif
		SonarModel(const sauron::Pose& sonarPose)
			: m_sonarX(sonarPose.X()),
			m_sonarY(sonarPose.Y()),
			m_sonarTheta(sonarPose.getTheta()),
			m_readings(20) {
		}

		SonarModel(pose_t x, pose_t y, pose_t theta)
			: m_sonarX(x), m_sonarY(y), m_sonarTheta(theta), m_readings(5) {
		}

		/** ISonarModel **/
		void addReading(const SonarReading& reading, const Pose& estimatedPose);
		bool validateReadings();

		bool tryGetMatchingMapLine(
			Map& map, // o mapa do ambiente
			double sigmaError2, // o erro utilizado no port�o de valida��o
			/*out*/LineSegment* matchedMapLine, // vari�vel de sa�da: a linha do mapa que foi associada, se alguma
			/*out*/ SonarReading* expectedReading, // vari�vel de sa�da: a leitura que seria esperada para aquela linha
			/*out*/SonarReading* actualReading // vari�vel de sa�da: a leitura que foi de fato obtida para aquela linha
		);
		/** ISonarModel **/

		Pose getSonarGlobalPose(const Pose& robotGlobalPose);
		Line getObservedLine();
		inline int getReadingsBufferCount() { return m_readings.size(); }
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
		bool robotHasTurned(const Pose& latestPose);
		struct ReadingAndPose {
			ReadingAndPose(const SonarReading& _reading, const Pose& _estimatedPose)
				: reading(_reading), estimatedPose(_estimatedPose) { }
			SonarReading reading;
			Pose estimatedPose;
		};

		bool isReadingMeaningful(const ReadingAndPose& readingAndPose);


		boost::circular_buffer<ReadingAndPose> m_readings;
#ifndef _CLR_
		// mutex que protege acesso a m_readings;
		boost::recursive_mutex m_readingsMutex;
#endif
		ReadingAndPose& getLatestReading();
		ReadingAndPose& getOldestReading();
	};

}
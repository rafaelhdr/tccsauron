#include "SonarModel.h"

#include <cassert>
#include <boost/numeric/ublas/matrix.hpp>

#include "MathHelper.h"
#include "Line.h"
#include "Cone.h"

// A definição _CLR_ é ativada quando o projeto está sendo compilado
// para testes unitários. Por algum motivo cósmico, a mera inclusão do
// header do mutex do Boost faz com que os testes não carreguem.
#ifndef _CLR_
#define SCOPED_READINGS_LOCK() boost::recursive_mutex::scoped_lock __lock__(m_readingsMutex)
#else
#define SCOPED_READINGS_LOCK()
#endif

namespace sauron
{

	void SonarModel::addReading(const SonarReading& reading, const Pose& estimatedPose) {
		SCOPED_READINGS_LOCK();
		if(robotHasTurned(estimatedPose)) {
			m_readings.clear();
		}
		ReadingAndPose rnp(reading, estimatedPose);
		if(isReadingMeaningful(rnp)) {
			m_readings.push_back(rnp);
		}
	}

	bool SonarModel::robotHasTurned(const Pose& latestPose) {
		SCOPED_READINGS_LOCK();
		if(m_readings.size() > 0) {
			return trigonometry::angularDistance(
				latestPose.Theta(),
				getOldestReading().estimatedPose.Theta()) >
				configs::sonars::minimumRobotTurnAngle;
		} else {
			return false;
		}
	}

	bool SonarModel::isReadingMeaningful(const ReadingAndPose& readingAndPose) {
		if(readingAndPose.reading.getReading() < configs::sonars::invalidReading) {
			if(m_readings.size() > 0) {
			pose_t distMoved = readingAndPose.estimatedPose.getDistance(getLatestReading().estimatedPose);
			return distMoved > configs::sonars::minimumRobotDistance;
			} else {
				return true;
			}
		} else {
			return false;
		}
	}

	double SonarModel::getSonarAngleOfIncidence() 
	{
		return this->m_sonarTheta + ::asin(getSinAlpha());
	}

	Line SonarModel::getObservedLine() {
		SCOPED_READINGS_LOCK();
		double beta_rads = getSonarAngleOfIncidence();

		Pose sonarPose = getSonarGlobalPose(getLatestReading().estimatedPose);
		pose_t thetaWall_rads = trigonometry::PI / 2 - beta_rads + sonarPose.getTheta() ;

		double reading = getLatestReading().reading * ::sin(beta_rads);
		double x_times_cos = sonarPose.X() * ::cos(thetaWall_rads);
		double y_times_sin = sonarPose.Y() * ::sin(thetaWall_rads);
		pose_t rWall = reading + x_times_cos + y_times_sin ;
		return Line(rWall, thetaWall_rads);
	}

	SonarReading SonarModel::getExpectedReadingByMapLine(const LineSegment& lineSegment)
	{
		SCOPED_READINGS_LOCK();
		sauron::Line line = lineSegment.getSauronLine();
		sauron::Pose sonarPose = this->getSonarGlobalPose(this->getLatestReading().estimatedPose);
		
		double beta_rads = getSonarAngleOfIncidence();

		double x_times_cos = sonarPose.X() * ::cos(line.getTheta());
		double y_times_sin = sonarPose.Y() * ::sin(line.getTheta());
		// expectedReading = (R_wall - X_sonar * cos Th_wall - Y_sonar * sin Th_wall) / sin Beta
		double expectedReading = (line.getRWall() - x_times_cos - y_times_sin) / ::sin(beta_rads);
		// HACK não sei ao certo por que às vezes a leitura esperada é negativa. Isso conserta
		// o teste SonarTest3::ExpectedReadingTest2_S3
		return expectedReading > 0 ? expectedReading : -expectedReading;
	}

	bool SonarModel::validateReadings()
	{
		SCOPED_READINGS_LOCK();
		/**
		* Algoritmo:
		* - verificar se o número de leituras, k, é >= a k_min
		* - calcular gamma para cada par de leituras
		* - obtém a variância e a média dos gammas
		* - calcula variancia_obsmedia
		* - calcula chi-quadrado dos gammas
		* - valida a hipótese
		* - obtém thetaWall e rWall
		**/
		int k = m_readings.size();
		if(k < configs::sonars::kMin) {
			return false;
		}

		/* gamma (pág 49 da tese do barra) é a razão entre a diferença do resultado
		dos sonares em duas leituras consecutivas e a distância percorrida entre
		essas leituras. Idealmente, esse valor se mantém constante no caso de obser-
		varmos um movimento retilíneo.

		como há k leituras, haverá k-1 gammas.
		*/
		const std::vector<double> gammas = getGammas();
		double gamma_mean = statistics::mean(gammas);
		double gamma_variance = statistics::sample_variance(gammas, gamma_mean);
		double obsmedia_variance = getObsMediaVariance();
		return statistics::chiSquareNormalDistributionTest(
			gammas.size(), gamma_variance, obsmedia_variance,
			configs::sonars::readingValidationAlpha);
	}

	std::vector<double> SonarModel::getGammas() {
		SCOPED_READINGS_LOCK();
		int k = m_readings.size();
		std::vector<double> gammas(k-1);
		// começa em 1 mesmo, porque fazemos m_readings[i] - m_readings[i-1]
		for(int i = 1; i < k; i++) {
			reading_t diff_readings = m_readings[i].reading - m_readings[i-1].reading;
			double diff_pose = m_readings[i].estimatedPose.getDistance(m_readings[i-1].estimatedPose);
			gammas.at(i-1) = diff_readings / diff_pose;
		}
		return gammas;
	}

	/** Página 50 da tese **/
	double SonarModel::getObsMediaVariance() {
		using namespace boost::numeric;
		SCOPED_READINGS_LOCK();
		double sin_alpha = getSinAlpha();
		assert(!(sin_alpha > 1.0 || sin_alpha < -1.0));
		double s2_d = getS2_D();
		double s2_r = getS2_R();

		ublas::matrix<double> eqCovar(2,2);
		eqCovar(0,0) = s2_d;
		eqCovar(0,1) = s2_d * sin_alpha;
		eqCovar(1,0) = s2_d * sin_alpha;
		eqCovar(1,1) = 2 * s2_r + s2_d * sin_alpha;

		ublas::vector<double> F(2);
		double d_robot = getD_Robot();
		F(0) = -1.0 * (m_readings.size() - 1) * getD_Sonar() / (d_robot * d_robot);
		F(1) = (m_readings.size() - 1) / d_robot;

		ublas::vector<double> prod = ublas::prod(eqCovar, F);
		return algelin::scalarProduct(prod, F);
	}

	double SonarModel::getSinAlpha() {
		SCOPED_READINGS_LOCK();
		// sin(alpha) = 1 / (d_robot / d_sonar), onde
		//	d_robot = distância percorrida pelo robô = 
		//		dist_euclideana(última_posição_estimada,primeira_posição_estimada)
		//	d_sonar = última_leitura_sonar - primeira_leitura_sonar
		// Note que o significado de "último" aqui é "mais recente" (o oposto à tese).
		double d_sonar = getD_Sonar();// * ::cos(this->m_sonarTheta);
		double d_robot = getD_Robot();

		// cosine law
		double x = ::sqrt(d_sonar * d_sonar + d_robot * d_robot - 
			2 * d_sonar * d_robot * ::cos(this->m_sonarTheta));
		// sine law
		double sinAlpha = d_sonar * ::sin(this->m_sonarTheta) / x;

		return trigonometry::correctImprecisions(sinAlpha);
	}


	bool SonarModel::tryGetMatchingMapLine(Map& map, double sigmaError2,
		LineSegment* matchedMapLine, SonarReading* expectedReading, SonarReading*
		actualReading)
	{
		SCOPED_READINGS_LOCK();
		std::vector<LineSegment>* pLines = map.getLines();	

		std::vector<LineSegment> mapLines = filterFarAwayLines(*pLines,
			getLatestReading().estimatedPose);
		mapLines = filterBySonarAngle(mapLines, getLatestReading().estimatedPose);
		
		std::vector<LineSegment> matchedLines;
		SonarReading latestReading = getLatestReading().reading;
		for(std::vector<LineSegment>::const_iterator it = mapLines.begin();
			it != mapLines.end(); it++) {
				if(matchMapLineWithReading(latestReading, *it, sigmaError2)) {
					matchedLines.push_back(*it);
				}
		}

		if(matchedLines.size() == 1) {
			if(matchedMapLine != NULL)
				*matchedMapLine = matchedLines[0];
			if(expectedReading != NULL)
				*expectedReading = getExpectedReadingByMapLine(matchedLines[0]);
			if(actualReading != NULL)
				*actualReading = getLatestReading().reading;
			return true;
		} else {
			return false;
		}
	}

	std::vector<LineSegment> SonarModel::filterFarAwayLines(std::vector<LineSegment>& mapLines,
		const Pose& pose) {
			std::vector<LineSegment> closeEnoughLines;
			for(std::vector<LineSegment>::iterator it = mapLines.begin(); it != mapLines.end();
				it++) {
					 double distanceToPose = it->getDistToLine(pose);
					 if(floating_point::equalOrLess(distanceToPose,
						 configs::sonars::maximumSonarToLineDistance))
					 {
						 closeEnoughLines.push_back(*it);
					 }
			}
			return closeEnoughLines;
	}

	std::vector<LineSegment> SonarModel::filterBySonarAngle(std::vector<LineSegment>& mapLines,
		const Pose& robotPose) {
			std::vector<LineSegment> reachableLines;
			Pose sonarPose = getSonarGlobalPose(robotPose);
			Cone sonarCone(sonarPose, sonarPose.getTheta(), configs::sonars::sonarApertureAngleRads);
			for(std::vector<LineSegment>::iterator it = mapLines.begin(); it != mapLines.end();
				it++) {
					if((floating_point::isEqual(
						it->getEndPoint1().getX(), 4375) && floating_point::isEqual(
						it->getEndPoint1().getY(), 40)) ||
						(floating_point::isEqual(
						it->getEndPoint2().getX(), 4375) && floating_point::isEqual(
						it->getEndPoint2().getY(), 40))){
							int a = 42;
					}
					if(sonarCone.intersectsSegment(*it)) {
						 reachableLines.push_back(*it);
					 }
			}
			return reachableLines;
	}

	bool SonarModel::matchMapLineWithReading(const SonarReading &reading,
		const LineSegment &mapLine, double sigmaError2) {
			double expectedReading =  getExpectedReadingByMapLine(mapLine);
			double v_2 = reading.getReading() - expectedReading;
			v_2 *= v_2;
			double s_2 = sigmaError2;

			return v_2 / s_2 < configs::sonars::wallRejectionValue2;
	}



	double SonarModel::getD_Robot() {
		SCOPED_READINGS_LOCK();
		return getLatestReading().estimatedPose.getDistance(
			getOldestReading().estimatedPose);
	}

	double SonarModel::getD_Sonar() {
		SCOPED_READINGS_LOCK();
		return  getOldestReading().reading.getReading() -
			getLatestReading().reading.getReading();
	}

	double SonarModel::getS2_D() {
		SCOPED_READINGS_LOCK();
		double d_robot = getD_Robot();
		double s_d = d_robot * configs::sonars::phoErrorFront4mm / (m_readings.size() - 1);
		return s_d * s_d;
	}

	double SonarModel::getS2_R() {
		SCOPED_READINGS_LOCK();
		double s_r = getLatestReading().reading.getStdDeviationMm();
		return s_r * s_r;
	}

	Pose SonarModel::getSonarGlobalPose(const Pose& robotPose) {
		pose_t globalSonarX, globalSonarY, globalSonarTh;
		globalSonarX = robotPose.X() +
			(m_sonarX * ::cos(robotPose.getTheta()) - m_sonarY * ::sin(robotPose.getTheta()));
		globalSonarY = robotPose.Y() +
			(m_sonarX * ::sin(robotPose.getTheta()) + m_sonarY * ::cos(robotPose.getTheta()));
		globalSonarTh = robotPose.getTheta() + m_sonarTheta;

		return Pose(globalSonarX, globalSonarY, globalSonarTh);
	}

	SonarModel::ReadingAndPose& SonarModel::getLatestReading() {
			SCOPED_READINGS_LOCK();
			return m_readings.back();
	}
	SonarModel::ReadingAndPose& SonarModel::getOldestReading() {
			SCOPED_READINGS_LOCK();
			return *m_readings.begin();
	}
}

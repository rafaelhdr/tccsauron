#include "Sonar.h"

#include <cassert>
#include <boost/numeric/ublas/matrix.hpp>

#include "MathHelper.h"
#include "Line.h"
#include "Cone.h"

namespace sauron
{

	void Sonar::addReading(const SonarReading& reading, const Pose& estimatedPose) {
		ReadingAndPose rnp(reading, estimatedPose);
		if(isReadingMeaningful(rnp)) {
			m_readings.push_back(rnp);
		}
	}

	double Sonar::getSonarAngleOfIncidence() 
	{
		return this->m_sonarTheta + ::asin(getSinAlpha());
	}

	Line Sonar::getObservedLine() {
		double beta_rads = getSonarAngleOfIncidence();

		Pose sonarPose = getSonarGlobalPose(getLatestReading().estimatedPose);
		pose_t thetaWall_rads = trigonometry::PI / 2 - beta_rads + sonarPose.getTheta() ;

		double reading = getLatestReading().reading * ::sin(beta_rads);
		double x_times_cos = sonarPose.X() * ::cos(thetaWall_rads);
		double y_times_sin = sonarPose.Y() * ::sin(thetaWall_rads);
		pose_t rWall = reading + x_times_cos + y_times_sin ;
		return Line(rWall, thetaWall_rads);
	}

	SonarReading Sonar::getExpectedReadingByMapLine(const LineSegment& lineSegment)
	{
		sauron::Line line = lineSegment.getSauronLine();
		sauron::Pose sonarPose = this->getSonarGlobalPose(this->getLatestReading().estimatedPose);
		
		double beta_rads = getSonarAngleOfIncidence();

		double x_times_cos = sonarPose.X() * ::cos(line.getTheta());
		double y_times_sin = sonarPose.Y() * ::sin(line.getTheta());
		// expectedReading = (R_wall - X_sonar * cos Th_wall - Y_sonar * sin Th_wall) / sin Beta
		return (line.getRWall() - x_times_cos - y_times_sin) / ::sin(beta_rads);
	}

	bool Sonar::validateReadings()
	{
		/**
		* Algoritmo:
		* - verificar se o n�mero de leituras, k, � >= a k_min
		* - calcular gamma para cada par de leituras
		* - obt�m a vari�ncia e a m�dia dos gammas
		* - calcula variancia_obsmedia
		* - calcula chi-quadrado dos gammas
		* - valida a hip�tese
		* - obt�m thetaWall e rWall
		**/
		int k = m_readings.size();
		if(k < configs::kMin) {
			return false;
		}

		/* gamma (p�g 49 da tese do barra) � a raz�o entre a diferen�a do resultado
		dos sonares em duas leituras consecutivas e a dist�ncia percorrida entre
		essas leituras. Idealmente, esse valor se mant�m constante no caso de obser-
		varmos um movimento retil�neo.

		como h� k leituras, haver� k-1 gammas.
		*/
		const std::vector<double> gammas = getGammas();
		double gamma_mean = statistics::mean(gammas);
		double gamma_variance = statistics::sample_variance(gammas, gamma_mean);
		double obsmedia_variance = getObsMediaVariance();
		return statistics::chiSquareNormalDistributionTest(
			gammas.size(), gamma_variance, obsmedia_variance, configs::readingValidationAlpha);
	}

	std::vector<double> Sonar::getGammas() {
		int k = m_readings.size();
		std::vector<double> gammas(k-1);
		// come�a em 1 mesmo, porque fazemos m_readings[i] - m_readings[i-1]
		for(int i = 1; i < k; i++) {
			reading_t diff_readings = m_readings[i].reading - m_readings[i-1].reading;
			double diff_pose = m_readings[i].estimatedPose.getDistance(m_readings[i-1].estimatedPose);
			gammas.at(i-1) = diff_readings / diff_pose;
		}
		return gammas;
	}

	/** P�gina 50 da tese **/
	double Sonar::getObsMediaVariance() {
		using namespace boost::numeric;

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

	double Sonar::getSinAlpha() {
		// sin(alpha) = 1 / (d_robot / d_sonar), onde
		//	d_robot = dist�ncia percorrida pelo rob� = 
		//		dist_euclideana(�ltima_posi��o_estimada,primeira_posi��o_estimada)
		//	d_sonar = �ltima_leitura_sonar - primeira_leitura_sonar
		// Note que o significado de "�ltimo" aqui � "mais recente" (o oposto � tese).
		double d_sonar = getD_Sonar();// * ::cos(this->m_sonarTheta);
		double d_robot = getD_Robot();

		// cosine law
		double x = ::sqrt(d_sonar * d_sonar + d_robot * d_robot - 
			2 * d_sonar * d_robot * ::cos(this->m_sonarTheta));
		// sine law
		double sinAlpha = d_sonar * ::sin(this->m_sonarTheta) / x;

		return trigonometry::correctImprecisions(sinAlpha);

		//return trigonometry::correctImprecisions(d_sonar / d_robot);
	}


	bool Sonar::tryGetMatchingMapLine(Map& map, LineSegment* matchedMapLine,
		SonarReading* expectedReading, double sigmaError2)
	{
		std::vector<LineSegment>* pLines = map.getLines();	

		std::vector<LineSegment> mapLines = filterFarAwayLines(*pLines,
			getLatestReading().estimatedPose);
		
		std::vector<LineSegment> matchedLines;
		SonarReading latestReading = getLatestReading().reading;
		for(std::vector<LineSegment>::const_iterator it = mapLines.begin();
			it != mapLines.end(); it++) {
				if(matchMapLineWithReading(latestReading, *it, sigmaError2)) {
					matchedLines.push_back(*it);
				}
		}

		if(matchedLines.size() == 1) {
			*matchedMapLine = matchedLines[0];
			*expectedReading = getExpectedReadingByMapLine(matchedLines[0]);
			return true;
		} else {
			return false;
		}
	}

	std::vector<LineSegment> Sonar::filterFarAwayLines(std::vector<LineSegment>& mapLines,
		const Pose& pose) {
			std::vector<LineSegment> closeEnoughLines;
			for(std::vector<LineSegment>::iterator it = mapLines.begin(); it != mapLines.end();
				it++) {
					 double distanceToPose = it->getDistToLine(pose);
					 if(floating_point::equalOrLess(distanceToPose, configs::maximalSonarToLineDistance))
					 {
						 closeEnoughLines.push_back(*it);
					 }
			}
			return closeEnoughLines;
	}

	std::vector<LineSegment> Sonar::filterBySonarAngle(std::vector<LineSegment>& mapLines,
		const Pose& robotPose) {
			std::vector<LineSegment> reachableLines;
			Pose sonarPose = getSonarGlobalPose(robotPose);
			Cone sonarCone(sonarPose, sonarPose.getTheta(), configs::sonars::sonarApertureAngleRads);
			for(std::vector<LineSegment>::iterator it = mapLines.begin(); it != mapLines.end();
				it++) {
					if(sonarCone.intersectsSegment(*it)) {
						 reachableLines.push_back(*it);
					 }
			}
			return reachableLines;
	}

	bool Sonar::matchMapLineWithReading(const SonarReading &reading,
		const LineSegment &mapLine, double sigmaError2) {
			if(floating_point::isEqual(mapLine.getSauronLine().getRWall(), 390)) {
				int a = 42;
			}
			double expectedReading =  getExpectedReadingByMapLine(mapLine);
			double v_2 = reading.getReading() - expectedReading;
			v_2 *= v_2;
			double s_2 = sigmaError2;

			return v_2 / s_2 < configs::wallRejectionValue2;
	}



	double Sonar::getD_Robot() {
		return getLatestReading().estimatedPose.getDistance(
			getOldestReading().estimatedPose);
	}

	double Sonar::getD_Sonar() {
		return  getOldestReading().reading.getReading() -
			getLatestReading().reading.getReading();
	}

	double Sonar::getS2_D() {
		double d_robot = getD_Robot();
		double s_d = d_robot * configs::phoErrorFront4mm / (m_readings.size() - 1);
		return s_d * s_d;
	}

	double Sonar::getS2_R() {
		double s_r = getLatestReading().reading.getStdDeviationMm();
		return s_r * s_r;
	}

	Pose Sonar::getSonarGlobalPose(const Pose& robotPose) {
		pose_t globalSonarX, globalSonarY, globalSonarTh;
		globalSonarX = robotPose.X() +
			(m_sonarX * ::cos(robotPose.getTheta()) - m_sonarY * ::sin(robotPose.getTheta()));
		globalSonarY = robotPose.Y() +
			(m_sonarX * ::sin(robotPose.getTheta()) + m_sonarY * ::cos(robotPose.getTheta()));
		globalSonarTh = robotPose.getTheta() + m_sonarTheta;

		return Pose(globalSonarX, globalSonarY, globalSonarTh);
	}
}

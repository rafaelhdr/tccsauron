#include "Sonar.h"

#include <cassert>
#include <boost/numeric/ublas/matrix.hpp>

#include "MathHelper.h"
#include "Line.h"

namespace sauron
{

	void Sonar::addReading(const SonarReading& reading, const Pose& estimatedPose) {
		ReadingAndPose rnp(reading, estimatedPose);
		if(isReadingMeaningful(rnp)) {
			m_readings.push_back(rnp);
		}
	}

	Line Sonar::getObservedLine() {
		double alpha_rads = ::asin(getSinAlpha());
		pose_t thetaWall_rads = getLatestReading().estimatedPose.getTheta() + alpha_rads;
		Pose sonarPose = getSonarGlobalPose(getLatestReading().estimatedPose);

		// DEBUG
		double reading = getLatestReading().reading * ::sin(sonarPose.getTheta());
		double x_times_sin = sonarPose.X() * ::sin(thetaWall_rads);
		double y_times_cos = sonarPose.Y() * ::cos(thetaWall_rads);
		pose_t rWall = reading + x_times_sin + y_times_cos ;
		return Line(rWall, thetaWall_rads);
	}

	bool Sonar::validateReadings()
	{
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
		if(k < configs::kMin) {
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
			gammas.size(), gamma_variance, obsmedia_variance, configs::readingValidationAlpha);
	}

	std::vector<double> Sonar::getGammas() {
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

		//return trigonometry::correctImprecisions(d_sonar / d_robot);
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

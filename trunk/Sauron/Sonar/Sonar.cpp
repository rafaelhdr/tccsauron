#include "Sonar.h"
#include "MathHelper.h"

namespace sauron
{
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
		double gamma_mean = array_math::mean(gammas);
	}

	std::vector<double> Sonar::getGammas() {
		int k = m_readings.size();
		std::vector<double> gammas(k-1);
		// começa em 1 mesmo, porque fazemos m_readings[i] - m_readings[i-1]
		for(int i = 1; i < k; i++) {
			float diff_readings = m_readings[i].reading - m_readings[i-1].reading;
			double diff_pose = m_readings[i].estimatedPose.getDistance(m_readings[i-1].estimatedPose);
			gammas.push_back(diff_readings / diff_pose);
		}
		return gammas;
	}
}

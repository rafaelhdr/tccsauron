#include "Sonar.h"
#include "MathHelper.h"

namespace sauron
{
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
		double gamma_mean = array_math::mean(gammas);
	}

	std::vector<double> Sonar::getGammas() {
		int k = m_readings.size();
		std::vector<double> gammas(k-1);
		// come�a em 1 mesmo, porque fazemos m_readings[i] - m_readings[i-1]
		for(int i = 1; i < k; i++) {
			float diff_readings = m_readings[i].reading - m_readings[i-1].reading;
			double diff_pose = m_readings[i].estimatedPose.getDistance(m_readings[i-1].estimatedPose);
			gammas.push_back(diff_readings / diff_pose);
		}
		return gammas;
	}
}

#include "MedidaOdometro.h"

namespace sauron 
{

	namespace modeloDinamica
	{


		pose_t MedidaOdometro::getDistance()
		{
			return distance;
		}

		pose_t MedidaOdometro::getTheta()
		{
			return theta;
		}

		MedidaOdometro MedidaOdometro::minus (const MedidaOdometro other) const
		{
			return MedidaOdometro(distance - other.distance, theta - other.theta);

		}

		
	}



}
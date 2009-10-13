#include "MedidaOdometro.h"

namespace sauron 
{

	namespace modeloDinamica
	{


		pose_t MedidaOdometro::getDistance() const
		{
			return distance;
		}

		pose_t MedidaOdometro::getTheta() const
		{
			return theta;
		}

		MedidaOdometro MedidaOdometro::minus (const MedidaOdometro other) const
		{
			return MedidaOdometro(distance - other.distance, theta - other.theta);

		}

		
	}



}
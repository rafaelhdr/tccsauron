#include "MedidaOdometro.h"

namespace sauron 
{

	namespace modeloDinamica
	{

		MedidaOdometro::MedidaOdometro(pose_t _distance, pose_t _theta)
		{
			distance = _distance;
			theta = _theta;
		}

		pose_t MedidaOdometro::getDistance()
		{
			return distance;
		}

		pose_t MedidaOdometro::getTheta()
		{
			return theta;
		}

		MedidaOdometro MedidaOdometro::minus (MedidaOdometro other)
		{
			return MedidaOdometro(distance - other.distance, theta - other.theta);

		}

		
	}



}
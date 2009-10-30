#include "MedidaOdometro.h"
#include <cmath>

namespace sauron 
{

	namespace modeloDinamica
	{

		

		MedidaOdometro::MedidaOdometro(ArRobot&   robot) : robot(robot)
		{
			oldDistance = robot.getOdometerDistance()*1000;
			oldTheta = robot.getOdometerDegrees()*M_PI/(double)180;
			atualizaMedida();
		}

		pose_t MedidaOdometro::getDeltaDistance()
		{
			return deltaDistance;
		}

		pose_t MedidaOdometro::getDeltaTheta()
		{
			return deltaTheta;
		}

		void MedidaOdometro::atualizaMedida()
		{
			double newDistance = robot.getOdometerDistance()*1000;
			deltaDistance = newDistance - oldDistance;
			oldDistance = newDistance;

			double newTheta = robot.getOdometerDegrees()*M_PI/(double)180;
			deltaTheta = newTheta - oldTheta;
			oldTheta = newTheta;

		}

		

		
	}



}
#include "MedidaOdometro.h"
#include "MathHelper.h"
#include <cmath>

namespace sauron 
{

	namespace modeloDinamica
	{
		MedidaOdometro::MedidaOdometro(ArRobot&   robot) : robot(robot)
		{
			oldDistance = robot.getOdometerDistance()*10;
			oldTheta = trigonometry::degrees2rads(robot.getOdometerDegrees());
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
			double newDistance = robot.getOdometerDistance()*10;
			deltaDistance = newDistance - oldDistance;
			oldDistance = newDistance;

			double newTheta = trigonometry::degrees2rads(robot.getOdometerDegrees());
			deltaTheta = newTheta - oldTheta;
			oldTheta = newTheta;
		}
	}
}
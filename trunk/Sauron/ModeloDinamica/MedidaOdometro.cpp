#include "MedidaOdometro.h"
#include "MathHelper.h"
#include <cmath>
#include "log.h"
#define ODOMETER_LOG(level) FILE_LOG(level) << "Odometro #" << ": "

namespace sauron 
{

	namespace modeloDinamica
	{
		MedidaOdometro::MedidaOdometro(ArRobot&   robot) : robot(robot)
		{
			oldDistance = robot.getOdometerDistance()/10;
			oldTheta = robot.getOdometerDegrees();
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
			double newDistance = robot.getOdometerDistance()/10;
			// deltaDistance é sempre positivo, odometro só cresce
			deltaDistance = newDistance - oldDistance;
			if(robot.getVel() < 0)
			{
				deltaDistance = -deltaDistance;
			}
			oldDistance = newDistance;
            
			double newTheta = robot.getOdometerDegrees();
			// deltaTheta também é sempre positivo
            deltaTheta = trigonometry::degrees2rads(newTheta - oldTheta);
            if(robot.getRotVel() < 0)
            {
                deltaTheta = -deltaTheta;
            }
			oldTheta = newTheta;
		}
	}
}
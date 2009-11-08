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
			deltaDistance = newDistance - oldDistance;
			oldDistance = newDistance;

            
            
			double newTheta = robot.getOdometerDegrees();
           

            if(!floating_point::isEqual(deltaTheta, 0))
            {
                ODOMETER_LOG(logDEBUG2) << "OldTheta: "<<oldTheta<< ". NewTheta: "<<newTheta;
            }

            deltaTheta = trigonometry::degrees2rads(newTheta - oldTheta);
            if(robot.getRotVel() < 0)
            {
                deltaTheta = -deltaTheta;
            }
			oldTheta = newTheta;
		}
	}
}
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
			double newDistance = robot.getOdometerDistance()/10;
			deltaDistance = newDistance - oldDistance;
			oldDistance = newDistance;

            
            double odometer = robot.getOdometerDegrees();
			double newTheta = trigonometry::degrees2rads(odometer);
            deltaTheta = newTheta - oldTheta;
            if(deltaTheta > trigonometry::PI)
                deltaTheta = -2*trigonometry::PI + deltaTheta;
            if(deltaTheta < -trigonometry::PI)
                deltaTheta = 2*trigonometry::PI + deltaTheta;
            
            if(!floating_point::isEqual(deltaTheta, 0))
            {
                ODOMETER_LOG(logDEBUG2) << "OldTheta: "<<oldTheta<< ". Odometer: "<<odometer<<". NewTheta: "<< newTheta;
            }
			oldTheta = newTheta;
		}
	}
}
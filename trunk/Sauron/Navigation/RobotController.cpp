#include "RobotController.h"
#include "Aria.h"
#include "Pose.h"
#include "MathHelper.h"
#include <boost/thread/thread.hpp>

namespace sauron
{
	namespace robotController
	{
		void setRobot(ArRobot* robot) {
			details::p_robot = robot;
			details::p_robot->setRotVelMax(15);
		}

		Pose getPose() {
			ArPose robotPose = details::p_robot->getPose();
			return sauron::Pose(robotPose.getX(), robotPose.getY(), robotPose.getTh());
		}

		inline boost::xtime delay(int secs, int msecs=0, int nsecs=0) 
		{ 
			const int MILLISECONDS_PER_SECOND = 1000; 
			const int NANOSECONDS_PER_SECOND = 1000000000; 
			const int NANOSECONDS_PER_MILLISECOND = 1000000; 


			boost::xtime xt;
			boost::xtime_get (&xt, boost::TIME_UTC);

			nsecs += xt.nsec; 
			msecs += nsecs / NANOSECONDS_PER_MILLISECOND; 
			secs += msecs / MILLISECONDS_PER_SECOND; 
			nsecs += (msecs % MILLISECONDS_PER_SECOND) * 
				NANOSECONDS_PER_MILLISECOND; 
			xt.nsec = nsecs % NANOSECONDS_PER_SECOND; 
			xt.sec += secs + (nsecs / NANOSECONDS_PER_SECOND); 


			return xt; 
		} 

		void turn(double radians) {
			double newHeading = trigonometry::normalizeAngle(
				trigonometry::degrees2rads(details::p_robot->getTh()) + radians);
			details::p_robot->setHeading(trigonometry::rads2degrees(newHeading));
			while(true) {
				if(!details::p_robot->isHeadingDone() ){
					boost::thread::sleep(delay(0,300,0));
				} else {
					if(floating_point::isEqual(details::p_robot->getRotVel(), 0)) {
						break;
					} else {
						boost::thread::sleep(delay(0,100,0));
					}
				}
			}
			details::p_robot->clearDirectMotion();
		}




	}
}

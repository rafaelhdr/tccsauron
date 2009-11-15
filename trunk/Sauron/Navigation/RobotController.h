#pragma once

class ArRobot;
namespace sauron
{
	class Pose;
	namespace robotController
	{
		void setRobot(ArRobot* robot);
		Pose getPose();
		void turn(double radians);

		namespace details
		{
			static ArRobot* p_robot;
		}
	};
}

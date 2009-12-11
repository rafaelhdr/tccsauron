#pragma once
#include "Pose.h"
#include <boost/thread.hpp>
#include "Sonar/SonarReading.h"
#include "ArFunctor.h"
class ArRobot;
namespace sauron
{
	class RobotController;
	class LocalizationManager;

	class RouteExecuter
	{
	public:
		enum MoveResult
		{
			SUCCESS,
			FAILED_STRAYED,
			FAILED_EMERGENCY_STOP,
			FAILED_OTHER
		};

		RouteExecuter(LocalizationManager* locManager);
		RouteExecuter(ArRobot* robot, LocalizationManager* locManager);
		~RouteExecuter(){}
		
		MoveResult goTo(const Point2DDouble& to);
		


	public:
		static double getTurnAngle(const Pose& from, const Point2DDouble& to);
		static double getTurnAngle(ArRobot* robot, const Point2DDouble& to);

	private:
		ArRobot* mp_robot;
		LocalizationManager* mp_localization;
	};
}

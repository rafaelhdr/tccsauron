#pragma once
#include "Pose.h"
#include <boost/thread.hpp>
#include "Sonar/SonarReading.h"

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
		
		MoveResult goTo(const Point2DDouble& to);
		void AvoidObstacle(int sonarNumber, sauron::SonarReading reading);


	public:
		static double getTurnAngle(const Pose& from, const Point2DDouble& to);
		static double getTurnAngle(ArRobot* robot, const Point2DDouble& to);

	private:
		ArRobot* mp_robot;
		LocalizationManager* mp_localization;

		boost::condition_variable m_movementStoppedCond;
		bool m_movementStopped;
		boost::mutex m_mutex;
		MoveResult m_moveResult;

		void reachedGoal(MoveResult result);
		void waitGoalIsReached();
	};
}

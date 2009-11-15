#pragma once
#include "Pose.h"
#include <boost/thread.hpp>
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

		RouteExecuter();
		RouteExecuter(ArRobot* robot);
		
		MoveResult goTo(LocalizationManager* locManager, const Point2DDouble& to);



	public:
		double getTurnAngle(const Pose& from, const Point2DDouble& to);
		double getTurnAngle(ArRobot* robot, const Point2DDouble& to);

	private:
		ArRobot* mp_robot;
		boost::condition_variable m_movementStoppedCond;
		bool m_movementStopped;
		boost::mutex m_mutex;
		MoveResult m_moveResult;

		void reachedGoal(MoveResult result);
		void waitGoalIsReached();
	};
}

#pragma once
#include "Aria.h"
#include <boost/thread/thread.hpp>
#include "RouteExecuter.h"
#include "ArRobot.h"


class ArRobot;
namespace sauron
{
	class LocalizationManager;
class CruiseControl
{
public:
	CruiseControl(ArRobot* robot, LocalizationManager* localization, const Point2DDouble& goal);
	~CruiseControl(void);

	RouteExecuter::MoveResult go();
	void halt();
	void setDistanceToObstacle(double distanceCm);
	
private:
	ArRobot* mp_robot;
	LocalizationManager* mp_localization;
	Point2DDouble m_goal;
	int m_poseChangedCallbackId;

	boost::thread m_takeoffThread;
	bool m_isInTakeoff;
	bool m_isApproaching;
	void takeoff();
	void cruiseFlight(const Pose& currentPose);

	ArLineSegment drawRoute();

	void waitUntilGoalIsReached();
	RouteExecuter::MoveResult m_moveResult;

	boost::condition_variable m_movementStoppedCond;
	bool m_movementStopped;
	boost::mutex m_mutex;

	void reachedGoal(RouteExecuter::MoveResult result);

	double m_maxSpeed;
	double m_cruiseControlSpeed;
	void setMaxSpeed(double speed);
	void setCruiseControlSpeed(double speed);
	void updateRobotSpeed();
};
}

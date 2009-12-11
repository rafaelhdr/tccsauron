#include "RouteExecuter.h"
#include "Pose.h"
#include "Aria.h"
#include "MathHelper.h"
#include "RobotController.h"
#include "PoseTracker.h"

#include "Localization/LocalizationManager.h"

namespace sauron
{
	class LocalizationManager;

	RouteExecuter::RouteExecuter(LocalizationManager* locManager) : mp_robot(0), mp_localization(locManager),
		 m_callback(this, &RouteExecuter::AvoidObstacle)
	{
		for(int i = 0; i < 8; i++)
		{
			locManager->getSonarDataProvider()->setAddReadingCallback(i, &m_callback);
		}
	}
	RouteExecuter::RouteExecuter(ArRobot* robot, LocalizationManager* locManager) : mp_robot(robot), mp_localization(locManager),
		 m_callback(this, &RouteExecuter::AvoidObstacle)
	{
		for(int i = 0; i < 8; i++)
		{
			locManager->getSonarDataProvider()->setAddReadingCallback(i, &m_callback);
		}
	}

	RouteExecuter::~RouteExecuter()
	{
		for(int i = 0; i < 8; i++)
		{
			mp_localization->getSonarDataProvider()->removeAddReadingCallback(i, &m_callback);
		}
	}


	

	void RouteExecuter::AvoidObstacle(int sonarNumber, sauron::SonarReading reading)
	{
		const int LIMIT = 50;

		if(!m_movementStopped && sonarNumber > 0 && sonarNumber < 7 && reading.getReading() < LIMIT)
		{
			m_moveResult = RouteExecuter::FAILED_EMERGENCY_STOP;
			{
				boost::lock_guard<boost::mutex> lock(m_mutex);
				m_movementStopped = true;
			}
			m_movementStoppedCond.notify_all();
		}

	}


	RouteExecuter::MoveResult RouteExecuter::goTo(const Point2DDouble& to)
	{
		Pose currentPose = mp_localization->getPose();

		double deltaHeading = getTurnAngle(currentPose, to);
		mp_localization->setIsTurning(true);
		robotController::turn(deltaHeading);
		mp_localization->setIsTurning(false);

		ArLineSegment route(currentPose.X(), currentPose.Y(), to.X(), to.Y());

		
		PoseTracker tracker(mp_localization, to, route);
		tracker.trackAsync(boost::bind(&RouteExecuter::reachedGoal, this, _1));
		mp_robot->setVel(500);
		waitGoalIsReached();
		mp_robot->stop();
		return m_moveResult;
	}

	void RouteExecuter::waitGoalIsReached() {
		m_movementStopped = false;
		boost::unique_lock<boost::mutex> lock(m_mutex);
		while(!m_movementStopped) {
			m_movementStoppedCond.wait(lock);
		}
	}


	

	void RouteExecuter::reachedGoal(MoveResult result) {
		m_moveResult = result;
		{
			boost::lock_guard<boost::mutex> lock(m_mutex);
			m_movementStopped = true;
		}
		m_movementStoppedCond.notify_all();
	}

	double RouteExecuter::getTurnAngle(const Pose& from, const Point2DDouble& to)
	{
		// segmento que os liga
		double routeDeltaX = to.X()- from.X();
		double routeDeltaY = to.Y()- from.Y();

        /*return - from.Theta() + atan2( routeDeltaY, routeDeltaX ) ;*/

		const int segmentLength = 400;
		double currentDeltaX = segmentLength * ::cos(from.Theta());
		double currentDeltaY = segmentLength * ::sin(from.Theta());

		double crossProductLen = currentDeltaX * routeDeltaY - routeDeltaX * currentDeltaY;
		double dotProduct = currentDeltaX * routeDeltaX + currentDeltaY * routeDeltaY;

		return ::atan2(crossProductLen, dotProduct);
	}

	double RouteExecuter::getTurnAngle(ArRobot* robot, const Point2DDouble& to)
	{
		return trigonometry::degrees2rads(robot->findDeltaHeadingTo(ArPose(to.X(), to.Y())));
	}
}

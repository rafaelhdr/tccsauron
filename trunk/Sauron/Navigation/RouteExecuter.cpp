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

	RouteExecuter::RouteExecuter(LocalizationManager* locManager) : mp_robot(0), mp_localization(locManager)
	{
	}
	RouteExecuter::RouteExecuter(ArRobot* robot, LocalizationManager* locManager) : mp_robot(robot), mp_localization(locManager)
	{
	}

	RouteExecuter::MoveResult RouteExecuter::goTo(const Point2DDouble& to)
	{
		Pose currentPose = mp_localization->getPose();

		double deltaHeading = getTurnAngle(currentPose, to);
		robotController::turn(deltaHeading);

		ArLineSegment route(currentPose.X(), currentPose.Y(), to.X(), to.Y());

		
		PoseTracker tracker(mp_localization, to, route);
		tracker.trackAsync(boost::bind(&RouteExecuter::reachedGoal, this, _1));
		mp_robot->setVel(200);
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
		double routeDeltaX = to.X() - from.X();
		double routeDeltaY = to.Y() - from.Y();

		// deltaX e deltaY da postura atual, com um deltaX arbitrário
		double currentDeltaX, currentDeltaY;
		double cosTheta = ::cos(from.Theta());
		if(floating_point::isEqual(cosTheta, 0)) {
			currentDeltaY = ::sin(from.Theta()) * 5; // +5 ou -5
			currentDeltaX = 0;
		} else if(floating_point::isEqual(::sin(from.Theta()), 0)) {
			currentDeltaY = 0;
			currentDeltaX = cosTheta * 5; // +5 ou -5
		} else {
			currentDeltaX = cosTheta < 0 ? -5 : 5;
			currentDeltaY = currentDeltaX * ::tan(from.Theta());
		}

		double crossProductLen = currentDeltaX * routeDeltaY - routeDeltaX * currentDeltaY;
		double dotProduct = currentDeltaX * routeDeltaX + currentDeltaY * routeDeltaY;

		return ::atan2(crossProductLen, dotProduct);
	}

	double RouteExecuter::getTurnAngle(ArRobot* robot, const Point2DDouble& to)
	{
		return trigonometry::degrees2rads(robot->findDeltaHeadingTo(ArPose(to.X(), to.Y())));
	}
}

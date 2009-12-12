#include "RouteExecuter.h"
#include "Pose.h"
#include "Aria.h"
#include "MathHelper.h"
#include "RobotController.h"
#include "PoseTracker.h"
#include "CruiseControl.h"
#include "ObstacleMonitor.h"
#include "Localization/LocalizationManager.h"

namespace sauron
{
	class LocalizationManager;

	RouteExecuter::RouteExecuter(LocalizationManager* locManager)
		: mp_robot(0), mp_localization(locManager), mp_cruiseControl(0)
	{
	}
	RouteExecuter::RouteExecuter(ArRobot* robot, LocalizationManager* locManager) : mp_robot(robot),
		mp_localization(locManager), mp_cruiseControl(0)
	{
	}


	RouteExecuter::MoveResult RouteExecuter::goTo(const Point2DDouble& to)
	{
		Pose currentPose = mp_localization->getPose();

		double deltaHeading = getTurnAngle(currentPose, to);
		mp_localization->setIsTurning(true);
		robotController::turn(deltaHeading);
		mp_localization->setIsTurning(false);
		if(!m_halt)
		{
			{
				boost::unique_lock<boost::mutex> lock(m_cruiseControlMutex);
				mp_cruiseControl = new CruiseControl(mp_robot, mp_localization, to);
			}
			ObstacleMonitor monitor(mp_localization, mp_cruiseControl);
			MoveResult result = mp_cruiseControl->go();
			{
				boost::unique_lock<boost::mutex> lock(m_cruiseControlMutex);
				delete mp_cruiseControl; mp_cruiseControl = 0;
			}
			return result;
		} else {
			m_halt = false;
			return FAILED_OTHER;
		}
	}

	void RouteExecuter::halt()
	{
		m_halt = true;
		boost::unique_lock<boost::mutex> lock(m_cruiseControlMutex);
		if(mp_cruiseControl != 0)
		{
			mp_cruiseControl->halt();
		}
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

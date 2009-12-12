#include "Aria.h"
#include <boost/date_time.hpp>
#include "log.h"
#include "Localization/LocalizationManager.h"
#include "CruiseControl.h"
#include "PoseTracker.h"

#define CC_LOG(level) FILE_LOG(level) << "CruiseControl: "

namespace sauron
{
	CruiseControl::CruiseControl(ArRobot* robot, LocalizationManager* localization, const Point2DDouble& goal)
		: mp_robot(robot), m_goal(goal), mp_localization(localization), m_maxSpeed(-1),
		m_cruiseControlSpeed(-1), m_poseChangedCallbackId(-1)
	{
	}

	RouteExecuter::MoveResult CruiseControl::go()
	{
		CC_LOG(logDEBUG1) << "Indo para " << Pose(m_goal);
		m_movementStopped = false;
		
		m_poseChangedCallbackId = mp_localization->addPoseChangedCallback(
			boost::bind(&CruiseControl::cruiseFlight, this, _1));

		m_isApproaching = false;
		m_takeoffThread = boost::thread::thread(&CruiseControl::takeoff, this);

		CC_LOG(logDEBUG1) << "esperando chegar no objetivo";
		waitUntilGoalIsReached();
		mp_robot->stop();
		CC_LOG(logDEBUG1) << "chegou com resultado = " << m_moveResult;
		return m_moveResult;
	}

	void CruiseControl::halt()
	{
		mp_robot->stop();
		m_moveResult = RouteExecuter::FAILED_OTHER;
		{
			boost::lock_guard<boost::mutex> lock(m_mutex);
			m_movementStopped = true;
		}
		m_movementStoppedCond.notify_all();
	}

	void CruiseControl::takeoff()
	{
		CC_LOG(logDEBUG1) << "decolando...";
		m_isInTakeoff = true;
		boost::posix_time::ptime startTime(boost::posix_time::microsec_clock::local_time());

		while(!m_movementStopped && !m_isApproaching)
		{
			boost::posix_time::ptime currentTime(boost::posix_time::microsec_clock::local_time());
			boost::posix_time::time_duration delta = 
				boost::posix_time::time_period(startTime, currentTime).length();
			if(delta.total_milliseconds() > 5000)
				break;
			else
			{
				// speed =  a = -8,7; b = 62,5; c = 4,17; d = 0; 0 <= x <= 5 segundos
				double x =  delta.total_milliseconds() / 1000.0;
				double speed = -8.7 * x * x * x + 62.5 * x * x + 4.17 * x;
				setCruiseControlSpeed(speed);
				CC_LOG(logDEBUG1) << "decolagem: v = " << speed << " (/\\t = " << x << ")";
			}
			boost::this_thread::sleep(boost::posix_time::milliseconds(300));
		}
		m_isInTakeoff = false;
		CC_LOG(logDEBUG1) << "terminou decolagem";
	}

	void CruiseControl::cruiseFlight(const Pose& currentPose)
	{
		if(m_movementStopped)
			return;

		m_isApproaching = false;
		double distToGoal = currentPose.getDistance(Pose(m_goal));
		if(distToGoal < 60) {
			m_isApproaching = true;
			// speed = a = 1/4260; b = 75/710; c = -95/213; d = 100; 0 <= x <= 60 cm
			double speed = 1.0/4260 * distToGoal * distToGoal * distToGoal +
				75.0/710 * distToGoal * distToGoal - 95.0/213 * distToGoal + 100;
			if(m_cruiseControlSpeed > speed) // s� atualiza se for para reduzir
			{
				setCruiseControlSpeed(speed);
				CC_LOG(logDEBUG1) << "aproximacao: v = " << speed << " (distToGoal = " << distToGoal << " cm)";
			}
		} else {
			m_isApproaching = false;
			if(!m_isInTakeoff)
			{
				setCruiseControlSpeed(500);
			}
		}
	}

	void CruiseControl::waitUntilGoalIsReached()
	{
		PoseTracker tracker(mp_localization, m_goal, drawRoute());
		tracker.trackAsync(boost::bind(&CruiseControl::reachedGoal, this, _1));

		boost::unique_lock<boost::mutex> lock(m_mutex);
		while(!m_movementStopped) {
			m_movementStoppedCond.wait(lock);
		}
	}

	ArLineSegment CruiseControl::drawRoute()
	{
		Pose currentPose = mp_localization->getPose();
		return ArLineSegment(currentPose.X(), currentPose.Y(), m_goal.X(), m_goal.Y());
	}

	void CruiseControl::reachedGoal(RouteExecuter::MoveResult result) {
		m_moveResult = result;
		{
			boost::lock_guard<boost::mutex> lock(m_mutex);
			m_movementStopped = true;
		}
		m_movementStoppedCond.notify_all();
	}

	void CruiseControl::setDistanceToObstacle(double distanceCm)
	{
		if(distanceCm < 70) {
			//Equa��o anticolis�o do tipo f(x) = a ln(bx + c): x em cm, 20 <= x <= 70
            //a = 262,879; b = 0,114; c = -1,28;
			double speed = 262.879 * ::log(0.114 * distanceCm -1.28);
			setMaxSpeed(speed);
			CC_LOG(logWARNING) << " obstacle avoidance: v = " << speed << " (dist = " << distanceCm << ")";
		} else {
			setMaxSpeed(500);
		}
	}

	void CruiseControl::setMaxSpeed(double speed) {
		if(floating_point::equalOrLess(speed, 0)) {
			CC_LOG(logERROR) << "PARADA ANTICOLISAO (v = " << speed << ")";
			reachedGoal(RouteExecuter::FAILED_EMERGENCY_STOP);
			m_maxSpeed = 0;
		} else {
			m_maxSpeed = speed;
		}
		updateRobotSpeed();
	}

	void CruiseControl::setCruiseControlSpeed(double speed)
	{
		m_cruiseControlSpeed = speed;
		updateRobotSpeed();
	}

	void CruiseControl::updateRobotSpeed()
	{
		if(!m_movementStopped)
		{
			double speedToSet = 0;
			if(m_maxSpeed > 0) {
				speedToSet = min(m_cruiseControlSpeed, m_maxSpeed);
			} else {
				speedToSet = m_cruiseControlSpeed;
			}

			if(speedToSet > 0) {
				mp_robot->setVel(speedToSet);
			} else {
				mp_robot->stop();
			}
		} else {
			mp_robot->stop();
		}
	}

	CruiseControl::~CruiseControl(void)
	{
		if(m_poseChangedCallbackId != -1)
		{
			mp_localization->removePoseChangedCallback(m_poseChangedCallbackId);
		}
	}
}

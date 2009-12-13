#include "Aria.h"
#include <boost/date_time.hpp>
#include "log.h"
#include "Localization/LocalizationManager.h"
#include "CruiseControl.h"
#include "PoseTracker.h"

#define CC_LOG(level) FILE_LOG(level) << "CruiseControl: "

namespace sauron
{
	static const int MAX_SPEED = 500;
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
		CC_LOG(logDEBUG1) << "decolando... (movementStopped = " << m_movementStopped << "; isApproaching = " << m_isApproaching << ")";
		m_isInTakeoff = true;
		boost::posix_time::ptime startTime(boost::posix_time::microsec_clock::local_time());

		while(!m_movementStopped && !m_isApproaching)
		{
			boost::posix_time::ptime currentTime(boost::posix_time::microsec_clock::local_time());
			boost::posix_time::time_duration delta = 
				boost::posix_time::time_period(startTime, currentTime).length();
			if(delta.total_milliseconds() > 2000) {
				CC_LOG(logDEBUG1) << "decolagem chegou ao fim ( /\\t = " << delta.total_milliseconds() / 1000.0 << ")";
				break;
			}
			else
			{
				//v(x) = -187,5x^2(x-3), 0 <= x <= 2
				double x =  delta.total_milliseconds() / 1000.0;
				double speed = -187.5 * x * x * (x -3);
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
			//Para aterrissagem, usar a função f(x) = ax^3+bx^2+cx+d com x em centí­metros, com 0 <= x <= 60 e 
			//a = -13/10000, b = 39/200, c = 0, d = 100
			double speed = -13/10000 * distToGoal * distToGoal * distToGoal +
				39/200 * distToGoal * distToGoal + 100;
			if(m_cruiseControlSpeed > speed) // só atualiza se for para reduzir
			{
				m_isApproaching = true;
				setCruiseControlSpeed(speed);
				CC_LOG(logDEBUG1) << "aproximacao: v = " << speed << " (distToGoal = " << distToGoal << " cm)";
			}
		} else {
			m_isApproaching = false;
			if(!m_isInTakeoff)
			{
				setCruiseControlSpeed(MAX_SPEED);
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
			//Equação anticolisão do tipo f(x) = a ln(bx + c): x em cm, 20 <= x <= 70
            //a = 262,879; b = 0,114; c = -1,28;
			double speed = 262.879 * ::log(0.114 * distanceCm -1.28);
			setMaxSpeed(speed);
			CC_LOG(logWARNING) << " obstacle avoidance: v = " << speed << " (dist = " << distanceCm << ")";
		} else {
			setMaxSpeed(MAX_SPEED);
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
		boost::unique_lock<boost::mutex> lock(m_speedMutex);
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


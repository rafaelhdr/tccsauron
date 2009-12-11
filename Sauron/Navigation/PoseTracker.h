#pragma once
#include "Aria.h"
#include <boost/function.hpp>
#include "Pose.h"
#include "RouteExecuter.h"

namespace sauron
{
class LocalizationManager;

class PoseTracker
{
public:
	PoseTracker(LocalizationManager* locManager) :
	  mp_localization(locManager), m_poseChangedCallbackId(-1){ }

	PoseTracker(LocalizationManager* locManager, const Point2DDouble goal, const ArLineSegment route):
	   mp_localization(locManager), m_goal(goal), m_route(route), m_poseChangedCallbackId(-1){ }

	   ~PoseTracker();

	void setGoal(const Point2DDouble& goal) { m_goal = goal; }
	void setRoute(const ArLineSegment& route) { m_route = route; }
	void trackAsync(boost::function<void (RouteExecuter::MoveResult)> callback);


private:
	LocalizationManager* mp_localization;
	Point2DDouble m_goal;
	ArLineSegment m_route;
	int m_poseChangedCallbackId;
	boost::function<void (RouteExecuter::MoveResult)> m_callback;

	void poseHasChanged(const Pose& newPose);
	bool hasReachedGoal(const Pose& pose);
	bool hasStrayedFromRoute(const Pose& pose);
};
}
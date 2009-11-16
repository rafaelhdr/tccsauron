#include "PoseTracker.h"
#include "Localization/LocalizationManager.h"

namespace sauron
{
	PoseTracker::~PoseTracker()
	{
		if(m_poseChangedCallbackId != -1) {
			mp_localization->removePoseChangedCallback(m_poseChangedCallbackId);
		}
	}
	void PoseTracker::trackAsync(boost::function<void (RouteExecuter::MoveResult)> callback) {
		m_callback = callback;
		m_poseChangedCallbackId = mp_localization->addPoseChangedCallback(boost::bind(&PoseTracker::poseHasChanged, this, _1));
	}

	void PoseTracker::poseHasChanged(const sauron::Pose &newPose)
	{
		if(hasReachedGoal(newPose)) {
			m_callback(RouteExecuter::SUCCESS);
		} else if(hasStrayedFromRoute(newPose)) {
			m_callback(RouteExecuter::FAILED_STRAYED);
		}
	}

	bool PoseTracker::hasReachedGoal(const Pose& pose)
	{
		const int minDistanceCm = 7;
		return pose.getDistance(m_goal) < minDistanceCm;
	}

	bool PoseTracker::hasStrayedFromRoute(const Pose& pose)
	{
		const int maxDistanceCm = 40;
		int distanceToRoute = m_route.getPerpDist(ArPose(pose.X(), pose.Y(), pose.Theta()));
		return distanceToRoute > maxDistanceCm ||distanceToRoute < 0;
	}
}
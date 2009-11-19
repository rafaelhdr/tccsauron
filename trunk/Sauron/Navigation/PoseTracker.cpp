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
		const int minDistanceCm = 20;
		return pose.getDistance(m_goal) < minDistanceCm;
	}

	bool PoseTracker::hasStrayedFromRoute(const Pose& pose)
	{
		const int maxDistanceCm = 40;
		int distanceToRoute = m_route.getPerpDist(ArPose(pose.X(), pose.Y(), pose.Theta()));

		if(distanceToRoute > maxDistanceCm)
			return true;

		// se distanceToRout < 0, não há intersecção perpendicular entre a postura atual e a rota,
		// então vemos se estamos perto dos endpoints (isso evita ficarmos eternamente presos)
		if(distanceToRoute < 0) {

			ArPose endpoint1 = m_route.getEndPoint1(), endpoint2 = m_route.getEndPoint2();
			double distanceToEndpoint1 = pose.getDistance(Pose(endpoint1.getX(), endpoint1.getY(), endpoint1.getTh()));
			double distanceToEndpoint2 = pose.getDistance(Pose(endpoint2.getX(), endpoint2.getY(), endpoint2.getTh()));

			return distanceToEndpoint1 > maxDistanceCm && distanceToEndpoint2 > maxDistanceCm;
		}

		return false;
	}
}
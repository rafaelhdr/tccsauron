#pragma once
#include "Aria.h"

namespace sauron
{
class SauronArRobot :
	public ArRobot
{
public:
	SauronArRobot(const char * name = NULL, bool ignored = true,
		   bool doSigHandle=true, 
		   bool normalInit = true, bool addAriaExitCallback = true);
	~SauronArRobot(void);

	ArPose getBestPose() {
		return m_truePoseAvailable ? getTruePose() : getPose();
	}

	ArPose getTruePose() {
		return m_truePose;
	}

	bool hasTruePose() {
		return m_truePoseAvailable;
	}

private:
	bool m_truePoseAvailable;
	ArPose m_truePose;
	ArFunctorC<SauronArRobot> m_userTaskFunctor;
	void userTask_querySimStat();
	ArRetFunctor1C<bool,SauronArRobot, ArRobotPacket*> m_packetHandlerFunctor;
	bool simStatPacketHandler(ArRobotPacket* pkt);
};
}

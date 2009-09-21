#include "SauronArRobot.h"
#include "ArSimStat.h"

namespace sauron
{
SauronArRobot::SauronArRobot(const char * name, bool ignored,
		   bool doSigHandle, 
		   bool normalInit, bool addAriaExitCallback)
		   : ArRobot(name, ignored, doSigHandle, normalInit, addAriaExitCallback),
		   m_truePoseAvailable(false),
		   m_userTaskFunctor(this, &SauronArRobot::userTask_querySimStat),
		   m_packetHandlerFunctor(this, &SauronArRobot::simStatPacketHandler)

{
	this->addUserTask("SIM_STAT query", 50, &m_userTaskFunctor);
	this->addPacketHandler(&m_packetHandlerFunctor);
}

void SauronArRobot::userTask_querySimStat() {
	this->comInt(ArCommands::SIM_STAT, 1);
}

bool SauronArRobot::simStatPacketHandler(ArRobotPacket* pkt) {
	  if(pkt->getID() != 0x62)
		  return false;

	  ArSimStat simStat(pkt);
	  m_truePose.setX(simStat.getRealX());
	  m_truePose.setY(simStat.getRealY());
	  m_truePose.setTh(simStat.getRealTh());

	  m_truePoseAvailable = true;
	  return true;
}

SauronArRobot::~SauronArRobot(void)
{
	this->remUserTask(&m_userTaskFunctor);
	this->remPacketHandler(&m_packetHandlerFunctor);
}
}

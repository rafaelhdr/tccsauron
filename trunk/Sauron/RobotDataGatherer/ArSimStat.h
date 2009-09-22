#pragma once
#include "Aria.h"
#include <string>
#include <sstream>
class ArSimStat
{
public:

	ArSimStat(ArRobotPacket* pkt) {
		if(pkt->getID() != 0x62) {
			throw std::invalid_argument("Nao e SIM_STAT: o id do pacote nao e 0x62");
		}

		std::string simulatorName = this->getNullTerminatedString(pkt);
		std::string simulatorVersion = this->getNullTerminatedString(pkt);
		m_flags = pkt->bufToUByte4();
		
		m_simInt = pkt->bufToUByte2();
		m_realInt = pkt->bufToUByte2();
		m_lastInt = pkt->bufToUByte2();
		
		m_realX = pkt->bufToByte4();
		m_realY = pkt->bufToByte4();
		m_realZ = pkt->bufToByte4();
		m_realTh = pkt->bufToByte4();

	}

	~ArSimStat(void)
	{
	}

	unsigned int getFlags() { return m_flags; }
	unsigned int getSimInterval() { return m_simInt; }
	unsigned int getRealInterval() { return m_realInt; }

	/// Posição real em X (mm)
	int getRealX() { return m_realX; }
	/// Posição real em Y (mm)
	int getRealY() { return m_realY; }
	/// Posição real em Z (?!) (mm) 
	int getRealZ() { return m_realZ; }
	/// Posição real angular (em graus)
	int getRealTh() { return m_realTh; }


private:
	std::string getNullTerminatedString(ArRobotPacket* pkt) {
		std::stringstream ss;
		char read = 0;
		while((read = pkt->bufToByte()) != 0) {
			ss << read;
		}
		return ss.str();
	}
	unsigned int m_flags, m_simInt, m_realInt, m_lastInt;
	int m_realX, m_realY, m_realZ, m_realTh;

};

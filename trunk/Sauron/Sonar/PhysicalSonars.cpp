#include "PhysicalSonars.h"
#include "SonarReading.h"
#include "ArRobot.h"

namespace sauron {

PhysicalSonars::PhysicalSonars(ArRobot* robot)
: mp_robot(robot), m_getReadingTask(this, &PhysicalSonars::getReading)
{
	mp_robot->addSensorInterpTask("sauron::PhysicalSonars", 50, &m_getReadingTask);
}

void PhysicalSonars::setAddReadingCallback(int sonarNumber,
										   AddReadingCallback* pCallback)
{
	boost::unique_lock<boost::mutex> lock(m_callbackMutex);
	m_addReadingCallbacks[sonarNumber].push_back(pCallback);
}

void PhysicalSonars::removeAddReadingCallback(int sonarNumber, AddReadingCallback* p_callback) {
	boost::unique_lock<boost::mutex> lock(m_callbackMutex);
	std::vector<AddReadingCallback*>::iterator it;
	for(it = m_addReadingCallbacks[sonarNumber].begin();
		it != m_addReadingCallbacks[sonarNumber].end(); it++) {
			if(*it == p_callback) {
				m_addReadingCallbacks[sonarNumber].erase(it);
				break;
			}
	}
}

void PhysicalSonars::removeCallback(int sonarNumber) {
	boost::unique_lock<boost::mutex> lock(m_callbackMutex);
	m_addReadingCallbacks[sonarNumber].clear();
}

void PhysicalSonars::removeAllCallbacks() {
	boost::unique_lock<boost::mutex> lock(m_callbackMutex);
	m_addReadingCallbacks.clear();
}

void PhysicalSonars::getReading()
{
	boost::unique_lock<boost::mutex> lock(m_callbackMutex);
	for(std::map<int,std::vector<AddReadingCallback*>>::const_iterator it =
		m_addReadingCallbacks.begin(); it != m_addReadingCallbacks.end(); it++) {
			int sonarNumber = it->first;

			for(std::vector<AddReadingCallback*>::const_iterator callIt = it->second.begin(); 
				callIt != it->second.end();
				callIt++
				)
			{
				AddReadingCallback* pCallback  = *callIt;

				if(pCallback != 0) {
					ArSensorReading* pReading = mp_robot->getSonarReading(sonarNumber);
					if(pReading != 0) {
						if(pReading->isNew(mp_robot->getCounter()) &&
							!pReading->getIgnoreThisReading()) {

								pCallback->invoke(sonarNumber, SonarReading(pReading->getRange() / 10));

						}
					}
				}
			}
	}
}

PhysicalSonars::~PhysicalSonars(void)
{
	boost::unique_lock<boost::mutex> lock(m_callbackMutex);
	removeAllCallbacks();
}

}
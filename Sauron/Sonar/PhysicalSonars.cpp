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
	m_addReadingCallbacks[sonarNumber].push_back(pCallback);
}

void PhysicalSonars::removeCallback(int sonarNumber) {
	m_addReadingCallbacks[sonarNumber].clear();
}

void PhysicalSonars::removeAllCallbacks() {
	m_addReadingCallbacks.clear();
}

void PhysicalSonars::getReading()
{
	for(std::map<int,std::vector<AddReadingCallback*>>::const_iterator it =
		m_addReadingCallbacks.begin(); it != m_addReadingCallbacks.end(); it++) {
			int sonarNumber = it->first;

			for(std::vector<AddReadingCallback*>::const_iterator callIt = it->second.begin(); 
					callIt != it->second.end();
					++callIt
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
	removeAllCallbacks();
}

}
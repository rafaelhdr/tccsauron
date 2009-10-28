#include "PhysicalSonars.h"
#include "SonarReading.h"
namespace sauron {

PhysicalSonars::PhysicalSonars(ArRobot* robot)
: mp_robot(robot), m_getReadingTask(this, &PhysicalSonars::getReading)
{
	mp_robot->addSensorInterpTask("sauron::PhysicalSonars", 50, &m_getReadingTask);
}

void PhysicalSonars::setAddReadingCallback(int sonarNumber,
										   ArFunctor1<SonarReading>* pCallback)
{
	m_addReadingCallbacks[sonarNumber] = pCallback;
}

void PhysicalSonars::removeCallback(int sonarNumber) {
	m_addReadingCallbacks[sonarNumber] = 0;
}

void PhysicalSonars::removeAllCallbacks() {
	m_addReadingCallbacks.clear();
}

void PhysicalSonars::getReading()
{
	for(std::map<int,ArFunctor1<SonarReading>*>::const_iterator it =
		m_addReadingCallbacks.begin(); it != m_addReadingCallbacks.end(); it++) {
			int sonarNumber = it->first;
			ArFunctor1<SonarReading>* pCallback  = it->second;
			if(pCallback != 0) {
				ArSensorReading* pReading = mp_robot->getSonarReading(sonarNumber);
				if(pReading != 0) {
					if(pReading->isNew(mp_robot->getCounter()) &&
						!pReading->getIgnoreThisReading()) {
						pCallback->invoke(SonarReading(pReading->getRange()));
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
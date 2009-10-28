#pragma once
#include <map>
#include "ArRobot.h"
#include "ArFunctor.h"

namespace sauron
{
class SonarReading;
class PhysicalSonars
{
public:
	PhysicalSonars(ArRobot* robot);
	void setAddReadingCallback(int sonarNumber, ArFunctor1<SonarReading>* p_callback);
	void removeCallback(int sonarNumber);
	void removeAllCallbacks();
	~PhysicalSonars(void);

private:
	void getReading();
	ArRobot* mp_robot;
	ArFunctorC<PhysicalSonars> m_getReadingTask;
	struct SonarFunctor {
		int sonarNumber; ArFunctor1<SonarReading>* p_functor;
	};
	std::map<int,ArFunctor1<SonarReading>*> m_addReadingCallbacks;
};
}

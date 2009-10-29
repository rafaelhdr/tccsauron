#pragma once
#include <map>
#include "ArFunctor.h"

#include "ISonarDataAsyncProvider.h"

class ArRobot;

namespace sauron
{
class SonarReading;
class PhysicalSonars : public ISonarDataAsyncProvider
{
public:
	PhysicalSonars(ArRobot* robot);
	void setAddReadingCallback(int sonarNumber, AddReadingCallback* p_callback);
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
	std::map<int, AddReadingCallback*> m_addReadingCallbacks;
};
}

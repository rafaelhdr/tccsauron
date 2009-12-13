#pragma once
#include <map>
#include <boost/thread.hpp>
#include "Aria.h"
#include "Sonar/SonarReading.h"
namespace sauron
{
	class LocalizationManager;
	class CruiseControl;
class ObstacleMonitor
{
public:
	ObstacleMonitor(LocalizationManager* localization, CruiseControl* cruiseControl);
	void stopUpdate();
	~ObstacleMonitor(void);

private:
	LocalizationManager* mp_localization;
	CruiseControl* mp_cruiseControl;
	bool m_stopUpdate;
	ArFunctor2C<ObstacleMonitor, int, sauron::SonarReading> m_callback;
	void monitorReading(int sonarNumber, sauron::SonarReading reading);
    boost::mutex m_stopUpdateMutex;
	void updateCruiseControlWithClosestObstacle();
	std::map<int, double> m_readings;

};
}
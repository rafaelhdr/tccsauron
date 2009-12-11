#pragma once
#include <map>
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
	~ObstacleMonitor(void);

private:
	LocalizationManager* mp_localization;
	CruiseControl* mp_cruiseControl;
	ArFunctor2C<ObstacleMonitor, int, sauron::SonarReading> m_callback;
	void monitorReading(int sonarNumber, sauron::SonarReading reading);

	void updateCruiseControlWithClosestObstacle();
	std::map<int, double> m_readings;
};
}
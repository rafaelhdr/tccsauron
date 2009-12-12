#include "ObstacleMonitor.h"
#include "CruiseControl.h"
#include "Localization/LocalizationManager.h"

namespace sauron
{
	ObstacleMonitor::ObstacleMonitor(LocalizationManager* localization,
		CruiseControl* cruiseControl)
		: mp_localization(localization),
		m_callback(this, &ObstacleMonitor::monitorReading),
		mp_cruiseControl(cruiseControl), m_stopUpdate(false)
	{
		for(int i = 0; i < 8; i++)
		{
			m_readings[i] = -1;
			mp_localization->getSonarDataProvider()->setAddReadingCallback(i, &m_callback);
		}
	}

	void ObstacleMonitor::monitorReading(int sonarNumber, SonarReading reading)
	{
		m_readings[sonarNumber] = reading.getReading();
		updateCruiseControlWithClosestObstacle();
	}

	void ObstacleMonitor::updateCruiseControlWithClosestObstacle()
	{
		if(!m_stopUpdate)
		{
		double minReading = -1;
		for(int i = 1; i < 7; i++)
		{
			if(m_readings[i] != -1)
			{
				if(m_readings[i] != -1 && (minReading < 0 || minReading > m_readings[i]))
					minReading = m_readings[i];
			}
		}

		if(!(minReading < 0)) {
			mp_cruiseControl->setDistanceToObstacle(minReading);
		}
		}
	}

	ObstacleMonitor::~ObstacleMonitor(void)
	{
		for(int i = 0; i < 8; i++)
		{
			mp_localization->getSonarDataProvider()->removeAddReadingCallback(i, &m_callback);
		}
	}
}

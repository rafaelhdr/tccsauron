#pragma once

#include <vector>

#include "ArFunctor.h"

#include "ISensorModel.h"
#include "Sonar/Map.h"
#include "Sonar/ISonarDataAsyncProvider.h"
#include "Sonar/SonarModel.h"
#include "Sonar/SonarReading.h"

namespace sauron
{
class SonarSet;
class ILocalizationManager;
class ISonarModel;
class SensorSonar : public ISensorModel
{
    public:
        SensorSonar(int sonarNumber, ILocalizationManager& localizationManager,
			ISonarDataAsyncProvider& readingsProvider);
        ~SensorSonar();

        bool getEstimate( const Pose &last, 
                          Matrix &hValue, Measure &z, Model &H, Covariance &R );

        bool checkNewEstimateAvailable();
private:
	void setupAsyncDataFeed();
	void addReadingToModel(int sonarNumber, SonarReading reading);

	int m_sonarNumber;
	ISonarDataAsyncProvider& m_dataProvider;
	ILocalizationManager& m_localization;
	SonarModel m_model;
	ArFunctor2C<SensorSonar, int, SonarReading> m_callback;
};

}   // namespace sauron
#pragma once

#include <vector>
#include <boost/function.hpp>
#include "ArFunctor.h"

#include "ISensorSonarModel.h"
#include "CallbackHandler.h"
#include "SonarMatch.h"


#include "Sonar/Map.h"
#include "Sonar/ISonarDataAsyncProvider.h"
#include "Sonar/SonarModel.h"
#include "Sonar/SonarReading.h"
#include "Sonar/LineSegment.h"

namespace sauron
{
class SonarSet;
class ILocalizationManager;
class ISonarModel;

class SensorSonar : public ISensorSonarModel
{
    public:
        SensorSonar(int sonarNumber, ISonarDataAsyncProvider& readingsProvider);
        ~SensorSonar();
		void setLocalizationManager(ILocalizationManager& locManager);

		bool hasMatch() const { return m_hasMatch; }
		SonarMatch getLatestMatch() const { return m_latestMatch; } 

private:
	void updateEstimate();
	bool getEstimate( const Pose& last, Matrix &hValue, Measure &z, Model &H, Covariance &R );
	bool getEstimate( Matrix &hValue, Measure &z, Model &H, Covariance &R );
	void setupAsyncDataFeed();
	void addReadingToModel(int sonarNumber, SonarReading reading);
	double getMatchVariance(int matchScore);

	int m_sonarNumber;
	ISonarDataAsyncProvider& m_dataProvider;
	ILocalizationManager* mp_localization;
	SonarModel m_model;
	ArFunctor2C<SensorSonar, int, SonarReading> m_callback;

	bool m_hasMatch;
	SonarMatch m_latestMatch;
};

}   // namespace sauron
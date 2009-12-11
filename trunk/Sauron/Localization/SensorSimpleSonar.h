#pragma once
#include "ISensorModel.h"
#include "Sonar/SimpleSonarModel.h"
#include "ArFunctor.h"
#include "Sonar/ISonarDataAsyncProvider.h"
#include "Sonar/SonarReading.h"

namespace sauron
{
	class ILocalizationManager;

	class SensorSimpleSonar : public ISensorModel
	{
	public:
		SensorSimpleSonar(int sonarNumber, ISonarDataAsyncProvider& readingsProvider);
		void setLocalizationManager(ILocalizationManager& locManager) {
			mp_localization = &locManager;
		}

		~SensorSimpleSonar(void);


	private:

		int m_sonarNumber;
		ISonarDataAsyncProvider& m_dataProvider;
		ILocalizationManager* mp_localization;
		SimpleSonarModel m_model;
		ArFunctor2C<SensorSimpleSonar, int, SonarReading> m_callback;

		void updateEstimate(double reading);

		bool getEstimate(/*out*/Matrix &hValue,
			/*out*/Measure &z,
			/*out*/Model &H,
			/*out*/Covariance &R );
		double getFancyCovariance(double expected, double actual);
		void readingAvailable(int sonarNumber, SonarReading reading);

	};
}
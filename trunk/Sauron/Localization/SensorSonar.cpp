#include "SensorSonar.h"

#include "ILocalizationManager.h"
#include "MathHelper.h"
#include "Sonar/Configs.h"
#include "Sonar/ISonarModel.h"
#include "Sonar/SonarReading.h"
#include "Sonar/Line.h"

namespace sauron
{
	SensorSonar::SensorSonar(int sonarNumber, ILocalizationManager& localizationManager,
		ISonarDataAsyncProvider& readingsProvider, ISonarModel& sonarModel)
		: m_sonarNumber(sonarNumber), m_dataProvider(readingsProvider),  m_model(sonarModel),
		m_localization(localizationManager), m_callback(this, &SensorSonar::addReadingToModel)
	{
		if(sonarNumber < 0 ||sonarNumber > 8) {
			throw std::invalid_argument("Numero invalido de sonar");
		}
		setupAsyncDataFeed();
	}

	SensorSonar::~SensorSonar()
	{
	}

	bool SensorSonar::getEstimate( const Pose &last, 
		Matrix &hValue, Measure &z, Model &H, Covariance &R )
	{
		SonarReading expectedReading, actualReading;
		LineSegment matchedLineSegment;
		if(m_model.tryGetMatchingMapLine(m_localization.getMap(), configs::sonars::validationGateSigma2,
			&matchedLineSegment, &expectedReading, &actualReading)) {

				hValue(0,0) = expectedReading;

				z(0,0) = actualReading;

				R(0,0) = configs::sonars::sonarReadingStandardDeviationMm;
				R(1,1) = configs::sonars::sonarReadingStandardDeviationMm;
				R(2,2) = configs::sonars::sonarReadingStandardDeviationMm;

				Line matchedLine = matchedLineSegment.getSauronLine();
				H(0,0) = -1.0 * ::cos(matchedLine.getTheta());
				H(0,1) = -1.0 * ::sin(matchedLine.getTheta());
				// H(0,2) = x'_sonar * sin(Theta_n - Theta_wall) + y'_sonar * cos (Theta_n - Theta_wall)
				double theta_diff = last.Theta() - matchedLine.getTheta();
				Pose sonarRelativePose = configs::sonars::getSonarPose(m_sonarNumber);
				pose_t x_sonar = sonarRelativePose.X();
				pose_t y_sonar = sonarRelativePose.Y();
				H(0,2) = x_sonar * ::sin(theta_diff) + y_sonar * ::cos(theta_diff);
				return true;
		} else {
			return false;
		}
	}

	bool SensorSonar::checkNewEstimateAvailable()
	{
		return m_model.tryGetMatchingMapLine(m_localization.getMap(), configs::sonars::validationGateSigma2,
			NULL, NULL, NULL);
	}

	void SensorSonar::setupAsyncDataFeed()
	{
		m_dataProvider.setAddReadingCallback(m_sonarNumber, &m_callback);
	}

	void SensorSonar::addReadingToModel(int sonarNumber, SonarReading reading)
	{
		if(sonarNumber == m_sonarNumber) {
			Pose currentEstimatedPose = m_localization.getPose();
			m_model.addReading(reading, currentEstimatedPose);
		} else {
			throw std::invalid_argument("addReadingToModel: sonarNumber invalido");
		}
	}

}   // namespace sauron
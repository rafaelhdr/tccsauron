#include "SensorSonar.h"

#include "log.h"
#define SONAR_LOG(level) FILE_LOG(level) << "SensorSonar #"<< m_sonarNumber << ": "
#include "ILocalizationManager.h"
#include "MathHelper.h"
#include "Sonar/Configs.h"
#include "Sonar/ISonarModel.h"
#include "Sonar/SonarReading.h"
#include "Sonar/Line.h"

namespace sauron
{
	SensorSonar::SensorSonar(int sonarNumber, ILocalizationManager& localizationManager,
		ISonarDataAsyncProvider& readingsProvider)
		: m_sonarNumber(sonarNumber), m_dataProvider(readingsProvider),
		m_localization(localizationManager), m_callback(this, &SensorSonar::addReadingToModel),
		m_model(configs::sonars::getSonarPose(sonarNumber))
	{
		SONAR_LOG(logINFO) << " Construtor";
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
		SONAR_LOG(logDEBUG4) << "getEstimate (lastPose: " << last << ")";
		if(m_model.validateReadings()) {
			SONAR_LOG(logDEBUG3) << "Validou leituras (k = " << m_model.getReadingsBufferCount() << ")";
			if(m_model.tryGetMatchingMapLine(m_localization.getMap(),
				configs::sonars::validationGateSigma2,
				&matchedLineSegment, &expectedReading, &actualReading)) {
					SONAR_LOG(logDEBUG2) << "Pegou segmento: (" << matchedLineSegment.getEndPoint1().getX() << ", "<< matchedLineSegment.getEndPoint1().getY() << ") -> (" << matchedLineSegment.getEndPoint2().getX() << ", "<< matchedLineSegment.getEndPoint2().getY() << "))";
					SONAR_LOG(logDEBUG2) << "Leitura esperada: " << expectedReading.getReading();
					SONAR_LOG(logDEBUG2) << "Leitura recebida: " << actualReading.getReading();

                    z.resize( 1, 1 );
                    R.resize( 1, 1 );
                    H.resize( 1, 3 );
                    hValue.resize( 1, 1 );

                    z.clear();
                    R.clear();
                    H.clear();
                    hValue.clear();

					hValue(0,0) = expectedReading;

					z(0,0) = actualReading;

					R(0,0) = configs::sonars::sonarReadingStandardDeviationMm;
					
					Line matchedLine = matchedLineSegment.getSauronLine();
					H(0,0) = -1.0 * ::cos(matchedLine.getTheta());
					H(0,1) = -1.0 * ::sin(matchedLine.getTheta());
					// H(0,2) = x'_sonar * sin(Theta_n - Theta_wall) + y'_sonar * cos (Theta_n - Theta_wall)
					double theta_diff = last.Theta() - matchedLine.getTheta();
					Pose sonarRelativePose = configs::sonars::getSonarPose(m_sonarNumber);
					pose_t x_sonar = sonarRelativePose.X();
					pose_t y_sonar = sonarRelativePose.Y();
					H(0,2) = x_sonar * ::sin(theta_diff) + y_sonar * ::cos(theta_diff);

					SONAR_LOG(logDEBUG3) << "Matriz H: " << H;
					return true;
			} else {
				SONAR_LOG(logDEBUG4) << "Não associou a segmento no mapa.";
			}
		} else {
			SONAR_LOG(logDEBUG4) << "Não validou leituras.";
		}
		return false;
	}

	bool SensorSonar::checkNewEstimateAvailable()
	{
		if(m_model.validateReadings())
			return m_model.tryGetMatchingMapLine(m_localization.getMap(), configs::sonars::validationGateSigma2,
			NULL, NULL, NULL);
		else
			return false;
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
#include "SensorSonar.h"

#include "log.h"
#define SONAR_LOG(level) FILE_LOG(level) << "SensorSonar #"<< m_sonarNumber << ": "

#include "Matrix.h"
#include "Pose.h"

#include "ILocalizationManager.h"
#include "MathHelper.h"
#include "Sonar/Configs.h"
#include "Sonar/ISonarModel.h"
#include "Sonar/SonarReading.h"
#include "Sonar/Line.h"

namespace sauron
{
	SensorSonar::SensorSonar(int sonarNumber,
		ISonarDataAsyncProvider& readingsProvider)
		: m_sonarNumber(sonarNumber), m_dataProvider(readingsProvider),
		mp_localization(0), m_callback(this, &SensorSonar::addReadingToModel),
		m_model(sonarNumber, configs::sonars::getSonarPose(sonarNumber))
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

	bool SensorSonar::getEstimate( Matrix &hValue, Measure &z, Model &H, Covariance &R )
	{
		SonarReading expectedReading, actualReading;
		LineSegment matchedLineSegment;
		const Pose last = mp_localization->getPose();
		SONAR_LOG(logDEBUG3) << "getEstimate (lastPose: " << last << ")";
		if(m_model.validateReadings()) {
			SONAR_LOG(logDEBUG3) << "Validou leituras (k = " << m_model.getReadingsBufferCount() << ")";
			if(m_model.tryGetMatchingMapLine(last, mp_localization->getMap(),
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
					Pose sonarRelativePose = configs::sonars::getSonarPose(m_sonarNumber);

					double denominator = ::sin(
						sonarRelativePose.Theta() + matchedLine.getTheta() - last.Theta() + trigonometry::PI / 2);

					double multiplier = trigonometry::normalizeAngle(
						m_model.getSonarGlobalPose(last).Theta()) < 0 ? 1.0 : -1.0;

					SONAR_LOG(logDEBUG2) << "Angulo do sonar: " << m_model.getSonarGlobalPose(last).Theta() <<
						"normalizado = " << trigonometry::normalizeAngle(m_model.getSonarGlobalPose(last).Theta()) <<
						"=> " << multiplier;

					//
					H(0,0) = multiplier * ::cos(matchedLine.getTheta());
					H(0,1) = multiplier * ::sin(matchedLine.getTheta());
					//H(0,1) = multiplier * ::sin(matchedLine.getTheta()) / denominator;
					// H(0,2) = x'_sonar * sin(Theta_n - Theta_wall) + y'_sonar * cos (Theta_n - Theta_wall)
					double theta_diff = last.Theta() - matchedLine.getTheta();

					pose_t x_sonar = sonarRelativePose.X();
					pose_t y_sonar = sonarRelativePose.Y();
					H(0,2) = x_sonar * ::sin(theta_diff) + y_sonar * ::cos(theta_diff);

					SONAR_LOG(logDEBUG3) << "Matriz H: " << H;
					return true;
			} else {
				SONAR_LOG(logDEBUG4) << "Não associou a segmento no mapa.";
			}
		} else {
			SONAR_LOG(logDEBUG4) << "Não validou leituras. (k = " << m_model.getReadingsBufferCount() << ")" ;
		}
		return false;
	}


	void SensorSonar::setupAsyncDataFeed()
	{
		m_dataProvider.setAddReadingCallback(m_sonarNumber, &m_callback);
	}

	void SensorSonar::setLocalizationManager(ILocalizationManager& locManager) {
			mp_localization = &locManager;
	}

	void SensorSonar::addReadingToModel(int sonarNumber, SonarReading reading)
	{
		if(sonarNumber == m_sonarNumber) {
			if(mp_localization != 0) {
				Pose currentEstimatedPose = mp_localization->getPose();
				if(m_model.addReading(reading, currentEstimatedPose)) {
					updateEstimate();
				}
			} else {
				SONAR_LOG(logWARNING) << "Leitura recebida, mas mp_localization é null";
			}
		} else {
			throw std::invalid_argument("addReadingToModel: sonarNumber invalido");
		}
	}

	void SensorSonar::updateEstimate()
	{
		if(mp_localization != 0) {
			Matrix          hValue; 
			Measure         z; 
			Model           H; 
			Covariance      R;
			if(getEstimate( hValue, z, H, R )) {
				mp_localization->update(hValue, z, H, R);
			}
		}
	}



}   // namespace sauron
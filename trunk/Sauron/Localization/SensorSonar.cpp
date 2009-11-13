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
		m_model(sonarNumber, configs::sonars::getSonarPose(sonarNumber)), m_hasMatch(false)
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
		bool matched = false;

		if(m_model.validateReadings()) {
			SONAR_LOG(logDEBUG2) << "Validou leituras (k = " << m_model.getReadingsBufferCount() << ")";
			if(m_model.tryGetMatchingMapLine(last, mp_localization->getMap(),
				configs::sonars::validationGateSigma2,
				&matchedLineSegment, &expectedReading, &actualReading)) {
					matched = true;
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
					
					if(floating_point::isEqual(denominator, 0, 0.01))
						return false;

					SONAR_LOG(logDEBUG2) << "Theta' = " << sonarRelativePose.Theta() << "; ThetaWall = " <<
						matchedLine.getTheta() << "; Theta = " << last.Theta() << "; soma = " <<
						sonarRelativePose.Theta() + matchedLine.getTheta() - last.Theta() + trigonometry::PI / 2
						<< "sin(soma) = denominator = " << denominator;

					double multiplier = trigonometry::normalizeAngle(
						m_model.getSonarGlobalPose(last).Theta()) < 0 ? 1.0 : -1.0;

					//SONAR_LOG(logDEBUG2) << "Angulo do sonar: " << m_model.getSonarGlobalPose(last).Theta() <<
					//	"normalizado = " << trigonometry::normalizeAngle(m_model.getSonarGlobalPose(last).Theta()) <<
					//	"=> " << multiplier;

					//
					H(0,0) = -1 * ::cos(matchedLine.getTheta()) / denominator;
					//H(0,0) = 0;
					H(0,1) = ::sin(matchedLine.getTheta()) / denominator;
					//H(0,0) = multiplier * ::cos(matchedLine.getTheta());
					//H(0,1) = multiplier * ::sin(matchedLine.getTheta());
					//H(0,1) = multiplier * ::sin(matchedLine.getTheta()) / denominator;
					// H(0,2) = x'_sonar * sin(Theta_n - Theta_wall) + y'_sonar * cos (Theta_n - Theta_wall)
					// novo:
					// h_antigo = x'_sonar * sin(Theta_n - Theta_wall) + y'_sonar * cos (Theta_n - Theta_wall)
					// angulo_novo = theta'_sonar + theta_wall - theta_n + pi/2
					// esperada = r_wall - (x_n + x'_sonar * cos(Theta_n) - y'_sonar * sen(Theta_n)) * cos(Theta_wall)
					//			  - (y_n + x'_sonar * sen(Theta_n) + y' * cos(Theta_n)) * sen(Theta_wall)
					// H(0,2) = (h_antigo * sin(angulo_novo) + esperada * cos(angulo_novo)) / sen(angulo_novo)^2
					double theta_diff = last.Theta() - matchedLine.getTheta();

					pose_t x_sonar = sonarRelativePose.X();
					pose_t y_sonar = sonarRelativePose.Y();
					pose_t th_sonar = sonarRelativePose.Theta();
					//H(0,2) = x_sonar * ::sin(theta_diff) + y_sonar * ::cos(theta_diff);
					double h_antigo =  x_sonar * ::sin(theta_diff) + y_sonar * ::cos(theta_diff);
					double angulo_novo = th_sonar + matchedLine.getTheta() - last.Theta() + trigonometry::PI/2;
					double obs_esperada = matchedLine.getRWall() -
						( last.X() + x_sonar * cos(last.Theta()) - y_sonar * sin(last.Theta())) * cos(matchedLine.getTheta())
						- (last.Y() + x_sonar * sin(last.Theta()) + y_sonar * cos(last.Theta())) * sin(matchedLine.getTheta());
					double sin_angulo_novo_2 = sin(angulo_novo);
					sin_angulo_novo_2 *= sin_angulo_novo_2;

					H(0,2) = (h_antigo * sin(angulo_novo) + obs_esperada * cos(angulo_novo)) / sin_angulo_novo_2;

					SONAR_LOG(logDEBUG3) << "Matriz H: " << H;

					m_latestMatch =  SonarMatch(m_sonarNumber, matchedLineSegment);
			} else {
				SONAR_LOG(logDEBUG4) << "Não associou a segmento no mapa.";
			}
		} else {
			SONAR_LOG(logDEBUG2) << "Não validou leituras. (k = " << m_model.getReadingsBufferCount() << ")" ;
		}
		m_hasMatch = matched;
		return matched;
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
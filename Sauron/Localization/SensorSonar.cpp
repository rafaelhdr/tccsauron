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
		int matchScore;
		boost::unique_lock<boost::recursive_mutex> lock(*mp_localization->getPoseMutex());

		const Pose last = mp_localization->getPose();
		SONAR_LOG(logDEBUG3) << "getEstimate (lastPose: " << last << ")";
		bool matched = false;

		if(m_model.validateReadings()) {
			SONAR_LOG(logDEBUG2) << "Validou leituras (k = " << m_model.getReadingsBufferCount() << ")";
			double beta;
			if(m_model.tryGetMatchingMapLine(last, mp_localization->getMap(),
				configs::sonars::validationGateSigma2,
				&matchedLineSegment, &expectedReading, &actualReading, &matchScore, &beta)) {
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

					if(matchScore == -1) {
						R(0,0) = configs::sonars::sonarReadingStandardDeviationMm;
					}
					else {
						R(0,0) = getMatchVariance(matchScore);
						SONAR_LOG(logDEBUG2) << "getMatchVariance: vari�ncia de " << R(0,0) << " com score de " << matchScore;
					}
					
					Line matchedLine = matchedLineSegment.getSauronLine();
					Pose sonarRelativePose = configs::sonars::getSonarPose(m_sonarNumber);

					/*double denominator = ::sin(
						sonarRelativePose.Theta() + matchedLine.getTheta() - last.Theta() + trigonometry::PI / 2);*/

					beta = sonarRelativePose.Theta() + (trigonometry::PI / 2 - matchedLine.getTheta()) + last.Theta();

					if(floating_point::isEqual(::sin(beta), 0, 0.01))
						return false;

					SONAR_LOG(logDEBUG2) << "Theta' = " << sonarRelativePose.Theta() << "; ThetaWall = " <<
						matchedLine.getTheta() << "; Theta = " << last.Theta() << "; soma = " <<
						sonarRelativePose.Theta() + matchedLine.getTheta() - last.Theta() + trigonometry::PI / 2
						<< "sin(soma) = denominator = " << beta;

					double multiplier = trigonometry::normalizeAngle(
						m_model.getSonarGlobalPose(last).Theta()) < 0 ? 1.0 : -1.0;

					//SONAR_LOG(logDEBUG2) << "Angulo do sonar: " << m_model.getSonarGlobalPose(last).Theta() <<
					//	"normalizado = " << trigonometry::normalizeAngle(m_model.getSonarGlobalPose(last).Theta()) <<
					//	"=> " << multiplier;

					//
					H(0,0) = -1 * ::cos(matchedLine.getTheta()) / ::sin(beta);
					//H(0,0) = 0;
					H(0,1) = -1 * ::sin(matchedLine.getTheta()) / ::sin(beta);
					//H(0,0) = multiplier * ::cos(matchedLine.getTheta());
					//H(0,1) = multiplier * ::sin(matchedLine.getTheta());
					//H(0,1) = multiplier * ::sin(matchedLine.getTheta()) / denominator;
					// H(0,2) = x'_sonar * sin(Theta_n - Theta_wall) + y'_sonar * cos (Theta_n - Theta_wall)
					// novo:
					// h = f/g
					// queremos h' = df/dg (em rela��o a theta)

					// df = x'_sonar * sin(Theta_n - Theta_wall) + y'_sonar * cos (Theta_n - Theta_wall)

					// f = r_wall - (x_n + x'_sonar * cos(Theta_n) - y'_sonar * sen(Theta_n)) * cos(Theta_wall)
					//			  - (y_n + x'_sonar * sen(Theta_n) + y' * cos(Theta_n)) * sen(Theta_wall)
					// H(0,2) = (df * sin(beta) + f * cos(beta)) / sen(beta)^2
					double theta_diff = last.Theta() - matchedLine.getTheta();

					pose_t x_sonar = sonarRelativePose.X();
					pose_t y_sonar = sonarRelativePose.Y();
					pose_t th_sonar = sonarRelativePose.Theta();
					//H(0,2) = x_sonar * ::sin(theta_diff) + y_sonar * ::cos(theta_diff);
					double df =  x_sonar * ::sin(theta_diff) + y_sonar * ::cos(theta_diff);
					double f = matchedLine.getRWall() -
						( last.X() + x_sonar * cos(last.Theta()) - y_sonar * sin(last.Theta())) * cos(matchedLine.getTheta())
						- (last.Y() + x_sonar * sin(last.Theta()) + y_sonar * cos(last.Theta())) * sin(matchedLine.getTheta());

					H(0,2) = (df * sin(beta) - f * cos(beta)) / (::sin(beta) * ::sin(beta));
					//H(0,2) = df / ::sin(beta);

					SONAR_LOG(logDEBUG3) << "Matriz H: " << H;

					m_latestMatch =  SonarMatch(m_sonarNumber, matchedLineSegment);
			} else {
				SONAR_LOG(logDEBUG4) << "N�o associou a segmento no mapa.";
			}
		} else {
			SONAR_LOG(logDEBUG2) << "N�o validou leituras. (k = " << m_model.getReadingsBufferCount() << ")" ;
		}
		m_hasMatch = matched;
		return matched;
	}

	double SensorSonar::getMatchVariance(int matchScore)
	{
		if(floating_point::isEqual(matchScore, 0))
			return 500;
		return 10 + 300 / matchScore;
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
				SONAR_LOG(logWARNING) << "Leitura recebida, mas mp_localization � null";
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
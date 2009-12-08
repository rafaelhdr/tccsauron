#include "SensorSimpleSonar.h"
#include "ILocalizationManager.h"
#include "Line.h"
#include "Sonar/Configs.h"
#include "Map.h"
#include "log.h"

#define SONAR_LOG(level) FILE_LOG(level) << "SensorSimpleSonar #"<< m_sonarNumber << ": "

namespace sauron
{
	SensorSimpleSonar::SensorSimpleSonar(int sonarNumber, ISonarDataAsyncProvider& readingsProvider)
		: m_sonarNumber(sonarNumber), m_dataProvider(readingsProvider), m_model(sonarNumber),
		mp_localization(0), m_callback(this, &SensorSimpleSonar::readingAvailable)
	{
		m_dataProvider.setAddReadingCallback(m_sonarNumber, &m_callback);
	}

	void SensorSimpleSonar::readingAvailable(int sonarNumber, SonarReading reading)
	{
		if(sonarNumber == m_sonarNumber) {
			updateEstimate(reading.getReading());
		}
	}

	void SensorSimpleSonar::updateEstimate(double reading)
	{
		if(reading < configs::sonars::invalidReading)
			if(mp_localization != 0) {
				Matrix          hValue(1, 1); 
				Measure         z(1, 1); 
				Model           H(1, 3); 
				Covariance      R(1, 1);

				SONAR_LOG(logDEBUG2) << "Leitura real = " << reading;
				z(0,0) = reading;
				if(getEstimate( hValue, z, H, R )) {
					mp_localization->update(hValue, z, H, R);
				}
			}
	}

	bool SensorSimpleSonar::getEstimate(Matrix& hValue, Measure& z, Model& H, Covariance& R)
	{
		boost::unique_lock<boost::recursive_mutex> lock(*mp_localization->getPoseMutex());

		Pose last = mp_localization->getPose();
		std::pair<double, LineSegment> expectedReadingAndWall = m_model.getExpectedReading(
			mp_localization->getMap(),
			last);

		double& expectedReading = expectedReadingAndWall.first;
		LineSegment& matchedSegment = expectedReadingAndWall.second;
		Line matchedLine = matchedSegment.getSauronLine();

		SONAR_LOG(logDEBUG2) << "Leitura esperada = " << expectedReading << "; segmento = " << matchedSegment;

		if(expectedReading != -1) {
			hValue(0,0) = expectedReading;

			R(0,0) = 60;

			Pose sonarRelativePose = configs::sonars::getSonarPose(m_sonarNumber);
			double beta = sonarRelativePose.Theta() + (trigonometry::PI / 2 - matchedLine.getTheta()) + last.Theta();

			if(floating_point::isEqual(::sin(beta), 0, 0.01))
				return false;

			H(0,0) = -1 * ::cos(matchedLine.getTheta()) / ::sin(beta);
			H(0,1) = -1 * ::sin(matchedLine.getTheta()) / ::sin(beta);
			double theta_diff = last.Theta() - matchedLine.getTheta();

			pose_t x_sonar = sonarRelativePose.X();
			pose_t y_sonar = sonarRelativePose.Y();
			pose_t th_sonar = sonarRelativePose.Theta();
			double df =  x_sonar * ::sin(theta_diff) + y_sonar * ::cos(theta_diff);
			double f = matchedLine.getRWall() -
				( last.X() + x_sonar * cos(last.Theta()) - y_sonar * sin(last.Theta())) * cos(matchedLine.getTheta())
				- (last.Y() + x_sonar * sin(last.Theta()) + y_sonar * cos(last.Theta())) * sin(matchedLine.getTheta());

			H(0,2) = (df * sin(beta) - f * cos(beta)) / (::sin(beta) * ::sin(beta));

			return true;
		}
		else
		{
			return false;
		}
	}

	SensorSimpleSonar::~SensorSimpleSonar(void)
	{
	}

}

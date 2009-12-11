

namespace sauron
{

	class RouteExecuter;
	class SonarReading;

class ObstacleAvoider
{
    public:
		ObstacleAvoider(sauron::RouteExecuter& routeExecuter);
        void avoidObstacle( int sonarNumber, sauron::SonarReading reading);

	private:
		sauron::RouteExecuter& routeExecuter;
};

}   // namespace sauron


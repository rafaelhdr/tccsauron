#pragma once
#include <vector>
#include "Sonar/ISonarModel.h"
#include "Sonar/SonarModel.h"
#include "Sonar/Configs.h"
#include <boost/scoped_ptr.hpp>
namespace sauron
{
class SonarSet
{
public:

	SonarSet(void)
	{
		for(int i = 0; i < 8; i++) {
			m_sonars.push_back(boost::shared_ptr<SonarModel>(new SonarModel(configs::sonars::getSonarPose(i))));
		}
	}

	inline ISonarModel& getSonar(int number) {
		if(number < 0 || number > 7) {
			throw std::invalid_argument("Numero de sonar invalido");
		}
		return *m_sonars.at(number);
	}

	inline int getSonarCount() {
		return m_sonars.size();
	}

private:
	std::vector<boost::shared_ptr<SonarModel> > m_sonars;
};
}

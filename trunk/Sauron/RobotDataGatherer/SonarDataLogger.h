#pragma once
#include "Aria.h"
#include "SauronArRobot.h"
#include "MathHelper.h"
#include <string>
#include <iostream>
#include <fstream>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

class SonarDataLogger
{
	public:
		SonarDataLogger(sauron::SauronArRobot *r, std::ofstream& file) :
		  robot(*r), robotTaskFunc(this, &SonarDataLogger::logTask),
		  logfile(file), m_stop(true){
			  if(!logfile.is_open()) {
				  std::cerr << "Arquivo nao esta aberto. " << GetLastError() << std::endl;
				  throw std::invalid_argument("logfile");
			  }
			  // add our task functor to the robot object as a user task, 
			  // to be invoked in every robot task cycle (approx. every 100ms):
			  robot.addSensorInterpTask("Logger", 50, &robotTaskFunc);    
		  }

		  ~SonarDataLogger() {
			  // it is important to remove our task if this object is destroyed, otherwise 
			  // ArRobot will hold an invalid ArFunctor pointer in its tasks list, resulting
			  // in a crash when it tries to invoke it.
			  robot.remSensorInterpTask(&robotTaskFunc);
		  }
		  void toggle() {
			  if(m_stop) start();
			  else stop();
		  }
		  void start() {
			  m_stop = false;
		  }
		  void stop() {
			  m_stop = true;
		  }
private:
	std::ofstream& logfile;
	bool m_stop;
	sauron::SauronArRobot& robot;
	ArTime lastLogTime;
	ArFunctorC<SonarDataLogger> robotTaskFunc;
	void logTask() {
		if(!m_stop) {
			logfile << getReadings(8) << std::endl << getPose() << std::endl;
			std::cout << std::endl << getReadings(8) << std::endl << getPose() << std::endl;
		}
	}

	std::string getReadings(int numOfSonars) {
		std::stringstream ss;
		for(int i = 0; i < numOfSonars; i++) {
			ss << robot.getSonarRange(i) / 10.0 << " ";
		}
		return ss.str();
	}

	std::string getPose() {
		std::stringstream ss;
		ArPose truePose = robot.getBestPose();
		ss << truePose.getX()<< " " << truePose.getY()<< " " <<
			truePose.getThRad();
		if(robot.hasTruePose())
			ss << " (TRUE)";
		else
			ss << " (ESTIMATED)";
		return ss.str();
	}
}; 
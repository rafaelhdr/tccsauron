#pragma once
#include "Aria.h"
#include "SauronArRobot.h"
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
			ss << robot.getSonarRange(i) << ";";
		}
		return ss.str();
	}

	std::string getPose() {
		std::stringstream ss;
		ArPose truePose = robot.getBestPose();
		ss << truePose.getX() / 1000.0 << ";" << truePose.getY() / 1000.0 << ";" << truePose.getTh();
		if(robot.hasTruePose())
			ss << "; (TRUE)";
		else
			ss << "; (ESTIMATED)";
		return ss.str();
	}
}; 

	/*
public:
	SonarDataLogger(const ArRobot& robot, const std::string& filename)
		: m_robot(robot), m_filename(filename), m_isLogging(false),
		m_loggerThread(0), m_logger(0), m_logfile(filename.c_str(), std::ios::out | std::ios::trunc){
			if(!m_logfile.is_open()) {
				throw std::invalid_argument("não foi possível abrir " + filename);
			}
	}
	~SonarDataLogger(void) {
		m_logfile.close();
	}

	void startLogging();
	void stopLogging();
	void toggleLogging();

private:
	bool m_isLogging;
	const ArRobot& m_robot;
	std::string m_filename;
	std::ofstream m_logfile;
	boost::scoped_ptr<boost::thread> m_loggerThread;

	class LoggerThread {
	public:
		LoggerThread(const ArRobot& _robot, std::ofstream& _logfile)
			: stop(false), robot(_robot), logfile(_logfile) {
				if(!logfile.is_open()) {
					throw std::invalid_argument("logfile");
				}
		}
		void operator()();
		void logReadings();
		std::string getReadings(int numOfSonars);
		std::string getPose();
		bool stop;
		const ArRobot& robot;
		std::ofstream& logfile;
	};


	boost::scoped_ptr<LoggerThread> m_logger;
};
*/
#pragma once
#include "Sonar.h"
#include "SonarReading.h"
#include "CustomTypes.h"
#include <cassert>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

struct LogLine {
	std::vector<sauron::SonarReading> readings;
	sauron::Pose pose;
};

class SonarReadingsLogParser
{
public:
	SonarReadingsLogParser(std::string logfilename)
		: m_log(logfilename.c_str()){
		if(!m_log.is_open()) {
			throw std::invalid_argument("O arquivo " + logfilename + " nao existe");
		}
		parse();
	}

	void addAllReadingsOfOneSonar(int sonarIndex, sauron::Sonar& sonar) const {
		for(std::vector<LogLine>::const_iterator it = m_readings.begin();
			it != m_readings.end(); it++) {
				sonar.addReading(it->readings.at(sonarIndex), it->pose);
		}
	}

	std::ifstream m_log;
	std::vector<LogLine> m_readings;

	void parse() {
		while(!m_log.eof()) {
			std::string sonarLine, poseLine;
			// a primeira linha contém a leitura dos sonares
			std::getline(m_log, sonarLine);
			// ignora linhas que comecem com '#'
			if(sonarLine.length() > 0 && sonarLine.at(0) != '#') {
				if(!m_log.eof()) {
					std::getline(m_log, poseLine);
					std::stringstream ss;
					ss << sonarLine;
					LogLine logLine;
					sauron::reading_t sonarReading;
					while(!ss.eof()) {
						ss >> sonarReading;
						if(!ss.fail())
							logLine.readings.push_back(sauron::SonarReading(sonarReading));
					}
					// a segunda tem a posição do robô
					sauron::pose_t x, y, th;
					ss.clear();
					ss.str(poseLine);
					ss >> x >> y >> th;
					logLine.pose.X() = x;
					logLine.pose.Y() = y;
					logLine.pose.setTheta(th);

					m_readings.push_back(logLine);
				}
			}
		}

	}


};
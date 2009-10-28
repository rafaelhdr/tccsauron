#pragma once
#include "CustomTypes.h"
#include "Configs.h"

namespace sauron
{
	class SonarReading
	{

	public:
		SonarReading() : m_reading(-1){ }
		SonarReading(reading_t reading) : m_reading(reading) {
		}

		reading_t getStdDeviationMm() const {
			return configs::sonarReadingStandardDeviationMm;
		}

		reading_t getReading() const {
			return m_reading;
		}

		operator reading_t () { return m_reading; }

	private:
		reading_t m_reading;
	};
}

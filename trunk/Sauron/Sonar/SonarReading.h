#pragma once
#include "CustomTypes.h"
#include "Configs.h"

namespace sauron
{
	class SonarReading
	{

	public:
		SonarReading(reading_t reading) : m_reading(reading) {
		}

		reading_t getStdDeviationMm2() {
			return configs::sonarReadingStandardDeviationMm2;
		}

		reading_t getReading() {
			return m_reading;
		}

		operator reading_t () { return m_reading; }

	private:
		reading_t m_reading;
	};
}

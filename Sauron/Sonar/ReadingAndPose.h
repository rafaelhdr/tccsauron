#pragma once

#include "SonarReading.h"
#include "Pose.h"

namespace sauron {
	struct ReadingAndPose {
		ReadingAndPose(const SonarReading& _reading, const Pose& _estimatedPose)
			: reading(_reading), estimatedPose(_estimatedPose) { }
		SonarReading reading;
		Pose estimatedPose;
	};
}


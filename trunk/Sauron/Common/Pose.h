#pragma once
#include "BaseTypes.h"
#include "CustomTypes.h"
#include "Point2D.h"

namespace sauron
{
	class Pose : public Point2D<pose_t>
	{
	public:
		Pose(void) : m_theta(0){
		}

		Pose(pose_t x, pose_t y, pose_t theta) :
			Point2D(x, y), m_theta(theta) {
				
		}

		pose_t getTheta() const { return m_theta; }
		void setTheta(pose_t theta) { m_theta = theta; }

	private:
		pose_t m_theta;
	};
}
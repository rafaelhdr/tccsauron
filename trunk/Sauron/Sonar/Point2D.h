#pragma once
#include "CustomTypes.h"
#include <cmath>

namespace sauron
{
class Point2D
{
public:
	Point2D(void) : m_x(0), m_y(0){
		}

		Point2D(pose_t x, pose_t y) :
		m_x(x), m_y(y){
		}

		pose_t getX() const { return m_x; }
		pose_t getY() const { return m_y; }

		void setX(pose_t x) { m_x = x; }
		void setY(pose_t y) { m_y = y; }

		double getDistance(const Point2D& point) const {
			double diff_x2 = getX() - point.getX(); diff_x2 *= diff_x2;
			double diff_y2 = getY() - point.getY(); diff_y2 *= diff_y2;
			return ::sqrt(diff_x2 + diff_y2);
		}

	private:
		pose_t m_x;
		pose_t m_y;
};
}

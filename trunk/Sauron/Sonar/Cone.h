#pragma once
#include "Point2D.h"
#include "ariaUtil.h"
#include "LineSegment.h"

namespace sauron
{
class Cone
{
public:
	Cone(const Point2DDouble& vertex, double axisAngleRads, double apertureAngleRads)
		: m_vertex(vertex), m_axisAngleRads(axisAngleRads),
		m_apertureAngleRads(apertureAngleRads){
			makeNewBorders();
	}

    inline ArLineSegment getBorder1() { return m_border1; }
	inline ArLineSegment getBorder2() { return m_border2; }

	bool intersectsSegment(const LineSegment& segment);

private:
	Point2DDouble getBorderPoint(const Point2DDouble& vertex, double lineAngle_rads);
	void makeNewBorders();
	Point2DDouble m_vertex;
	LineSegment m_border1, m_border2;
    ArLine m_axis;
	double m_axisAngleRads;
	double m_apertureAngleRads;
	 
};
}

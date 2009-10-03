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

	inline ArLine getBorder1() { return m_border1; }
	inline ArLine getBorder2() { return m_border2; }

	bool intersectsSegment(const LineSegment& segment) {
		const ArLine* line = segment.getLine();
		ArPose intersectionBorder1, intersectionBorder2;
		bool intersectsBorder1 = m_border1.intersects(line, &intersectionBorder1);
		bool intersectsBorder2 = m_border2.intersects(line, &intersectionBorder2);
		if((intersectsBorder1 && segment.linePointIsInSegment(&intersectionBorder1))
			|| (intersectsBorder2 && segment.linePointIsInSegment(&intersectionBorder2))){
				return true;
			}
		if(intersectsBorder1 && intersectsBorder2) {
			LineSegment segmentBetweenIntersections(
				intersectionBorder1.getX(), intersectionBorder1.getY(),
				intersectionBorder2.getX(), intersectionBorder2.getY());
			return segmentBetweenIntersections.contains(segment);
		}

		throw std::invalid_argument("Não foi possivel determinar");
	}
private:
	void makeNewBorders() {
		// constroi as duas retas de borda
		// as retas têm um ponto no vértice, e o outro é calculado

		// definimos um /\x = 5, e vemos o /\y necessário
		// primeiro para a reta inferior
		double deltaX = 5;
		double deltaY = deltaX * ::tan(m_axisAngleRads - m_apertureAngleRads / 2);
		Point2DDouble point1(m_vertex.X() + deltaX, m_vertex.Y() + deltaY);

		// agora para a reta superior
		deltaX = 5;
		deltaY = deltaX * ::tan(m_axisAngleRads + m_apertureAngleRads / 2);
		Point2DDouble point2(m_vertex.X() + deltaX, m_vertex.Y() + deltaY);

		// agora criamos as linhas
		m_border1.newParametersFromEndpoints(m_vertex.X(), m_vertex.Y(), point1.X(), point1.Y());
		m_border2.newParametersFromEndpoints(m_vertex.X(), m_vertex.Y(), point2.X(), point2.Y());

	}

private:
	Point2DDouble m_vertex;
	ArLine m_border1, m_border2;
	double m_axisAngleRads;
	double m_apertureAngleRads;
	 
};
}

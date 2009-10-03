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
			makeAxis();
	}

	inline ArLine getBorder1() { return m_border1; }
	inline ArLine getBorder2() { return m_border2; }

	bool intersectsSegment(const LineSegment& segment) {
		/*if(segment.isInLine(m_border1) || segment.isInLine(m_border2)) {
			return true;
		}*/
		const ArLine* line = segment.getLine();
		ArPose intersectionBorder1, intersectionBorder2;
		bool intersectsBorder1 = m_border1.intersects(line, &intersectionBorder1);
		bool intersectsBorder2 = m_border2.intersects(line, &intersectionBorder2);

		if(intersectsBorder1 && intersectsBorder2 && (intersectionBorder1 == intersectionBorder2))
			intersectsBorder2 = false;

		if((intersectsBorder1 && segment.linePointIsInSegment(&intersectionBorder1))
			|| (intersectsBorder2 && segment.linePointIsInSegment(&intersectionBorder2))){
				return true;
			}
		if(intersectsBorder1 && intersectsBorder2) {
			LineSegment segmentBetweenIntersections(
				intersectionBorder1.getX(), intersectionBorder1.getY(),
				intersectionBorder2.getX(), intersectionBorder2.getY());
			return segmentBetweenIntersections.contains(segment);
		} else if(intersectsBorder1 || intersectsBorder2) {
			ArLine lineEndpoint1Vertex = ArLine(m_vertex.X(), m_vertex.Y(),
				segment.getEndPoint1().getX(), segment.getEndPoint1().getY());
			double angleEndpoint1Vertex = getLineAngle(m_axis)-getLineAngle(lineEndpoint1Vertex);

			ArLine lineEndpoint2Vertex = ArLine(m_vertex.X(), m_vertex.Y(),
				segment.getEndPoint2().getX(), segment.getEndPoint2().getY());
			double angleEndpoint2Vertex = getLineAngle(m_axis)-getLineAngle(lineEndpoint2Vertex);

			return angleEndpoint1Vertex < (m_apertureAngleRads / 2)
				|| angleEndpoint2Vertex < (m_apertureAngleRads / 2);
		}

		throw std::invalid_argument("Não foi possivel determinar");
	}
private:
	double getLineAngle(const ArLine& line) const {
		return ::atan(-1.0 * line.getA() / line.getB());
	}
	void makeNewBorders() {
		// constroi as duas retas de borda
		// as retas têm um ponto no vértice, e o outro é calculado

		// definimos um /\x = 5, e vemos o /\y necessário
		// primeiro para a reta inferior
		Point2DDouble point1 = getBorderPoint(m_vertex,  m_axisAngleRads - m_apertureAngleRads / 2);
		Point2DDouble point2 = getBorderPoint(m_vertex, m_axisAngleRads + m_apertureAngleRads / 2);

		// agora criamos as linhas
		m_border1.newParametersFromEndpoints(m_vertex.X(), m_vertex.Y(), point1.X(), point1.Y());
		m_border2.newParametersFromEndpoints(m_vertex.X(), m_vertex.Y(), point2.X(), point2.Y());

	}

	void makeAxis() {
		Point2DDouble pointOnAxis = getBorderPoint(m_vertex, m_axisAngleRads);
		m_axis.newParametersFromEndpoints(m_vertex.X(), m_vertex.Y(), pointOnAxis.X(), pointOnAxis.Y());
	}

	Point2DDouble getBorderPoint(const Point2DDouble& vertex, double lineAngle_rads) {
		if(!floating_point::isEqual(lineAngle_rads, trigonometry::PI / 2)) {
			double deltaX = 5;
			double deltaY = deltaX * ::tan(lineAngle_rads);
			return Point2DDouble(m_vertex.X() + deltaX,  m_vertex.Y() + deltaY);
		} else {
			return Point2DDouble(m_vertex.X(),  m_vertex.Y() + 5);
		}
	}

private:
	Point2DDouble m_vertex;
	ArLine m_border1, m_border2, m_axis;
	double m_axisAngleRads;
	double m_apertureAngleRads;
	 
};
}

#include "Cone.h"

namespace sauron
{
	void Cone::makeNewBorders() {
		// constroi as duas retas de borda
		// as retas têm um ponto no vértice, e o outro é calculado

		// definimos um /\x = 5, e vemos o /\y necessário
		// primeiro para a reta inferior
		Point2DDouble point1 = getBorderPoint(m_vertex,  m_axisAngleRads - m_apertureAngleRads / 2);
		Point2DDouble point2 = getBorderPoint(m_vertex, m_axisAngleRads + m_apertureAngleRads / 2);

		// agora criamos as linhas

        m_border1 = LineSegment(m_vertex.X(), m_vertex.Y(), point1.X(), point1.Y());
		m_border2 = LineSegment(m_vertex.X(), m_vertex.Y(), point2.X(), point2.Y());
	}

	Point2DDouble Cone::getBorderPoint(const Point2DDouble& vertex, double lineAngle_rads) {
		double deltaX = 0;
		double cosLineAngle = ::cos(lineAngle_rads);
		if(!floating_point::isEqual(cosLineAngle, 0)) {
			if(cosLineAngle > 0) {
				// aumenta X
				deltaX = 1e20;
			} else if(cosLineAngle < 0) {
				// diminui X
				deltaX = -1e20;
			}
			double deltaY = deltaX * ::tan(lineAngle_rads);
			return Point2DDouble(m_vertex.X() + deltaX,  m_vertex.Y() + deltaY);
		} else {
			// pi/2 ou 3pi/2, só varia Y
			double deltaY = 0;
			double sinLineAngle = ::sin(lineAngle_rads);
			if(sinLineAngle > 0) {
				// aumenta Y
				deltaY = 1e20;
			} else {
				deltaY = -1e20;
			}
			return Point2DDouble(m_vertex.X(),  m_vertex.Y() + deltaY);
		}
	}

	bool Cone::intersectsSegment(const LineSegment& segment) {
		const ArLine* line = segment.getLine();
		ArPose intersectionBorder1, intersectionBorder2;

		bool intersectsBorder1 = m_border1.intersects(line, &intersectionBorder1);
		bool intersectsBorder2 = m_border2.intersects(line, &intersectionBorder2);

		if(intersectsBorder1 && intersectsBorder2 && (intersectionBorder1 == intersectionBorder2)) {
			intersectsBorder2 = false;
		}

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

			double angleEndpoint1Vertex = atan2(segment.getEndPoint1().getY(),
				segment.getEndPoint1().getX());

			double angleEndpoint2Vertex = atan2(segment.getEndPoint2().getY(),
				segment.getEndPoint2().getX());

			return floating_point::equalOrLess(
				trigonometry::angularDistance(m_axisAngleRads, angleEndpoint1Vertex),
					(m_apertureAngleRads / 2))
				|| floating_point::equalOrLess(
				trigonometry::angularDistance(m_axisAngleRads, angleEndpoint2Vertex),
					(m_apertureAngleRads / 2));

		} else {
			return false;
		}
	}
}
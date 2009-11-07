#pragma once
#include "MathHelper.h"
#include "ariaUtil.h"
#include "Line.h"
#include "Pose.h"

namespace sauron
{
	class LineSegment : public ArLineSegment
	{
	public:
		LineSegment() {
		}

		LineSegment(const ArLineSegment& segment) :
		ArLineSegment(segment.getX1() / 10, segment.getY1() / 10,
			segment.getX2() / 10, segment.getY2() / 10)
		{ }
		  LineSegment(double x1, double y1, double x2, double y2) :
		  ArLineSegment(x1, y1, x2, y2) { }
		Line getSauronLine() const
		{
			const ArLine* pline = this->getLine();
			double rWall = pline->getPerpDist(ArPose(0,0));
			ArPose endpoint1 = getEndPoint1();
			ArPose endpoint2  = getEndPoint2();
			double deltaX = endpoint1.getX() - endpoint2.getX();
			double deltaY = endpoint1.getY() - endpoint2.getY();
			double thWall;
			if(!floating_point::isEqual(deltaX, 0)) {
				ArPose intersection;
				pline->getPerpPoint(ArPose(0,0), &intersection);
				//thWall = trigonometry::PI / 2 + ::atan( deltaY / deltaX );
				thWall = ::atan2( intersection.getY(), intersection.getX() );
			} else {
				if(endpoint1.getX() > 0) {
					thWall = 0;
				} else {
					thWall = trigonometry::PI;
				}
			}
			return Line(rWall, thWall);
		}

		double getDistToLine(const Pose& pose) {
			ArPose arPose(pose.X(), pose.Y());
			arPose.setThRad(pose.getTheta());
			return ArLineSegment::getDistToLine(arPose);
		}

		bool contains(const LineSegment& segment) const {
			if(floating_point::isEqual(myX1, myX2) &&
				floating_point::isEqual(myY1, myY2)) {
					return floating_point::isEqual(segment.getX1(), segment.getX2())
						&& floating_point::isEqual(segment.getY1(), segment.getY2())
						&& floating_point::isEqual(segment.getX1(), myX1)
						&& floating_point::isEqual(segment.getY1(), myY1);
			}
			return linePointIsInSegment(&segment.getEndPoint1()) &&
				linePointIsInSegment(&segment.getEndPoint2());
		}
	};
}
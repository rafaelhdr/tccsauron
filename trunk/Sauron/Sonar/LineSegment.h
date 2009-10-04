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
		  ArLineSegment(segment) { }
		  LineSegment(double x1, double y1, double x2, double y2) :
		  ArLineSegment(x1, y1, x2, y2) { }
		Line getSauronLine() const
		{
			double rWall = this->getLine()->getPerpDist(ArPose(0,0)) / 10.0;
			ArPose endpoint1 = getEndPoint1();
			ArPose endpoint2  = getEndPoint2();
			double deltaX = endpoint1.getX() - endpoint2.getX();
			double deltaY = endpoint1.getY() - endpoint2.getY();
			double thWall;
			if(!floating_point::isEqual(deltaX, 0)) {
				thWall = trigonometry::PI / 2 + ::atan( deltaY / deltaX );
			} else {
				thWall = 0;
			}
			return Line(rWall, thWall);
		}

		double getDistToLine(const Pose& pose) {
			ArPose arPose(pose.X() * 10, pose.Y() * 10);
			arPose.setThRad(pose.getTheta());
			return ArLineSegment::getDistToLine(arPose) / 10;
		}

		bool contains(const LineSegment& segment) const {
			if(floating_point::isEqual(myX1, myX2) &&
				floating_point::isEqual(myY2, myY2)) {
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
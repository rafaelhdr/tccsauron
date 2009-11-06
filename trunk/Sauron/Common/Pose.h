#ifndef _POSE_H_
#define _POSE_H_

#include "BaseTypes.h"
#include "CustomTypes.h"
#include "Point2D.h"

#include <ostream>

namespace sauron
{
	class Pose 
	{
	    public:
		    Pose() : m_theta(0) {}

		    Pose( pose_t x, pose_t y, pose_t theta ) : m_position(x, y), m_theta(theta) { }

            pose_t &X()     { return m_position.X(); }
            pose_t &Y()     { return m_position.Y(); }
            pose_t &Theta() { return m_theta; }

            const pose_t &X() const     { return m_position.X(); }
            const pose_t &Y() const     { return m_position.Y(); }
            const pose_t &Theta() const { return m_theta; }

            double getDistance( const Pose &other ) const { return m_position.getDistance( other.m_position ); }

            // For backward compatibility
            operator Point2DDouble() { return m_position; }

            //*******************
            // Deprecated - use Theta() as setter and getter
            pose_t getTheta() const       { return m_theta; }
		    void setTheta( pose_t theta ) { m_theta = theta; }
            //*******************

	    private:
            pose_t            m_theta;
            Point2D< pose_t > m_position;

			friend std::ostream& operator<<(std::ostream& os, const Pose& p)
			{
				return os << '('<< p.X() << ", " << p.Y() << ", " << p.Theta() << ')';
			}

	};

}   // namespace sauron

#endif  // _POSE_H_
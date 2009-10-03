#pragma once
namespace sauron
{
class Line
{
public:
	Line(double rWall, double theta) 
		: m_rWall(rWall), m_theta(theta){
	}

	inline double getRWall() const { return m_rWall; }
	inline double getTheta() const { return m_theta; }

private:
	double m_rWall, m_theta;
};
}

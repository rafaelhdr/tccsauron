#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <boost/numeric/ublas/matrix.hpp>
#include "CustomTypes.h"
#include <iostream>
#include <iomanip>
namespace sauron
{

typedef boost::numeric::ublas::matrix< pose_t > Matrix;
typedef Matrix Model;
typedef Matrix Measure;
typedef Matrix Covariance;

	inline std::ostream& operator<<(std::ostream& os, const Matrix& m)
	{
		os << "[";
		for(unsigned int i = 0; i < m.size1(); i++) {
			for(unsigned int j = 0; j < m.size2(); j++) {
				os << std::setw(5) << m(i,j);
				if(j != m.size2() - 1) os << ", ";
			}
			if(i != m.size1() - 1) os << std::endl;
		}
		os << "]";
		return os;
	}

}   // namespace sauron

#endif // _MATRIX_H_
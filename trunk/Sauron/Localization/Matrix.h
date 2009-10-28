#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <boost/numeric/ublas/matrix.hpp>
#include "CustomTypes.h"

namespace sauron
{

typedef boost::numeric::ublas::matrix< pose_t > Matrix;
typedef Matrix Model;
typedef Matrix Measure;
typedef Matrix Covariance;

}   // namespace sauron

#endif // _MATRIX_H_
#ifndef GORILLA_MATH_H
#define GORILLA_MATH_H

#include <Eigen/Dense>
#include <float.h>

namespace gorilla {

/// \brief precision definitions
#ifndef SINGLE_PRECISION
typedef double real;
#define REAL_MAX DBL_MAX
#else
typedef float real;
#define REAL_MAX FLT_MAX
#endif

/// \brief Math types
typedef Eigen::Matrix<real, 3, 1> Vector3;

/// \brief Physics constants
#define GRAVITY Vector3(0, -9.81, 0)

} // namespace gorilla

#endif // GORILLA_MATH_H

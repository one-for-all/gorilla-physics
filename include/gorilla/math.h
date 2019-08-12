#ifndef GORILLA_PRECISION_H
#define GORILLA_PRECISION_H

#include <Eigen/Dense>

namespace gorilla {

/// \brief precision definitions
#ifndef SINGLE_PRECISION
    typedef double real;
#else
    typedef float real;
#endif

/// \brief Math types
typedef Eigen::Matrix<real, 3, 1> Vector3;

/// \brief Physics constants
#define GRAVITY Vector3(0, -9.81, 0)
}

#endif // GORILLA_PRECISION_H

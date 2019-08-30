#ifndef GORILLA_MATH_H
#define GORILLA_MATH_H

#include <Eigen/Dense>
#include <float.h>

namespace gorilla {

/// \brief precision definitions
#ifndef SINGLE_PRECISION
  typedef double real;
  #define REAL_MAX DBL_MAX
  #define real_pow pow
#else
  typedef float real;
  #define REAL_MAX FLT_MAX
  #define real_pw powf
#endif

/// \brief Vector of size 3
typedef Eigen::Matrix<real, 3, 1> Vector3;

/// \brief Vector of size 4
typedef Eigen::Matrix<real, 4, 1> Vector4;

/// \brief 3x3 Matrix
typedef Eigen::Matrix<real, 3, 3> Matrix3;

/// \brief 4x4 Matrix
typedef Eigen::Matrix<real, 4, 4> Matrix4;

/// \brief Quaternion
typedef Eigen::Quaternion<real> Quaternion;

/// \brief Updward unit vector
#define UP Vector3(0, 1, 0)

Matrix3 matrixLinearInterpolate(const Matrix3 &a, const Matrix3 &b,
                                const real amount);

} // namespace gorilla

#endif // GORILLA_MATH_H

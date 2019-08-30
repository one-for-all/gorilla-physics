#include <gorilla/math.h>

using namespace gorilla;

//////////////////////////////////////////////////////////////////////////
Matrix3 gorilla::matrixLinearInterpolate(const Matrix3 &a, const Matrix3 &b,
                                         const real amount)
{
  return a*(1-amount) + b*amount;
}
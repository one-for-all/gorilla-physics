#ifndef GORILLA_CORE_H
#define GORILLA_CORE_H

#include <math.h>

#include "precision.h"

namespace gorilla {
  class Vector3
  {
    public:
      real x;
      real y;
      real z;
    private:
      // Padding to ensure alignment
      real pad;
        
    public:
      Vector3() : x(0), y(0), z(0)
      {}

      Vector3(const real x, const real y, const real z):
        x(x), y(y), z(z) 
      {}

      const static Vector3 GRAVITY;

      void invert()
      {
        x = -x;
        y = -y;
        z = -z;
      }
      
      real magnitude() const
      {
        return real_sqrt(x*x + y*y + z*z);
      }

      real squareMagnitude() const
      {
        return x*x + y*y + z*z;
      }

      void normalize()
      {
        real l = magnitude();
        if (l > 0)
          (*this) *= 1.0/l;
      }

      void operator *=(const real value)
      {
        x *= value;
        y *= value;
        z *= value;
      }

      Vector3 operator *(const real value) const
      {
        return Vector3(x*value, y*value, z*value);
      }

      void operator +=(const Vector3 &v)
      {
        x += v.x;
        y += v.y;
        z += v.z;
      }

      Vector3 operator +(const Vector3 &v) const
      {
        return Vector3(x+v.x, y+v.y, z+v.z);
      }

      void operator -=(const Vector3 &v)
      {
        x -= v.x;
        y -= v.y;
        z -= v.z;
      }

      Vector3 operator -(const Vector3 &v) const
      {
        return Vector3(x-v.x, y-v.y, z-v.z);
      }

      real operator *(const Vector3 &v) const
      {
        return x*v.x + y*v.y + z*v.z;
      }

      Vector3 vectorProduct(const Vector3 &v) const
      {
        return Vector3(y*v.z - z*v.y,
                       z*v.x - x*v.z,
                       x*v.y - y*v.x);
      }

      void addScaledVector(const Vector3 &v, const real alpha)
      {
        x += alpha*v.x;
        y += alpha*v.y;
        z += alpha*v.z;
      }

      void clear()
      {
        x = y = z = 0;
      }
  };
}

#endif // GORILLA_CORE_H
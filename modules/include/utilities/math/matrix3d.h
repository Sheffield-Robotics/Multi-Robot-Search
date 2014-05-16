#ifndef MATRIX3D_H
#define MATRIX3D_H

#include "utilities/math/vector3d.h"

class Matrix3d {
   public:
      Matrix3d();
      Matrix3d(double a0,double a1,double a2,double a3,double a4,double a5,double a6,double a7,double a8);
      Matrix3d(double rotX, double rotY, double rotZ);
      double element[9];
      //Quaternion getQuaternion() const;
      double& operator () (int, int);
      Vector3d operator * (const Vector3d& v) const;
      Matrix3d operator * (const Matrix3d& v) const;
      Matrix3d getTransposed() const;
      void printMatrix() const;
      // [rk] the code operates on uninitialised memory => the returned values are total bullshit
      //double* getEulerAngles() const __attribute__((deprecated));
};

std::ostream& operator<<(std::ostream &os, const Matrix3d &m);

#endif

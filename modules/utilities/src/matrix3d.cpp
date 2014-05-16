#include "utilities/math/matrix3d.h"
#include "utilities/math/vector3d.h"
#include <iostream>
#include "math.h"
#include "stdio.h"

Matrix3d::Matrix3d(){
    element[0]=1;
    element[1]=0;
    element[2]=0;
    element[3]=0;
    element[4]=1;
    element[5]=0;
    element[6]=0;
    element[7]=0;
    element[8]=1;
}

Matrix3d::Matrix3d(double a0,double a1,double a2,double a3,double a4,double a5,double a6,double a7, double a8){
    element[0]=a0;
    element[1]=a1;
    element[2]=a2;
    element[3]=a3;
    element[4]=a4;
    element[5]=a5;
    element[6]=a6;
    element[7]=a7;
    element[8]=a8;
}

Matrix3d::Matrix3d(double rotX, double rotY, double rotZ){
   // FROM http://skal.planet-d.net/demo/matrixfaq.htm#Q31
   double A       = cos(rotX);
   double B        = sin(rotX);
   double C       = cos(rotY);
   double D       = sin(rotY);
   double E       = cos(rotZ);
   double F       = sin(rotZ);
   double AD      =   A * D;
   double BD      =   B * D;
   element[0]  =   C * E;
   element[1]  =  -C * F;
   element[2]  =   D;
   element[3]  =  BD * E + A * F;
   element[4]  = -BD * F + A * E;
   element[5]  =  -B * C;
   element[6]  = -AD * E + B * F;
   element[7]  =  AD * F + B * E;
   element[8] =   A * C;
};


Vector3d Matrix3d::operator* (const Vector3d &p) const {
   Vector3d result;
   result.x = p.x * element[0] + p.y * element[1] + p.z * element[2];
   result.y = p.x * element[3] + p.y * element[4] + p.z * element[5];
   result.z = p.x * element[6] + p.y * element[7] + p.z * element[8];
   return result;
};

Matrix3d Matrix3d::operator* (const Matrix3d &mat) const {
   Matrix3d result;
   for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
        result(i,j) = element[0+(i*3)]*mat.element[0+j] + element[1+(i*3)]*mat.element[3+j] + element[2+(i*3)]*mat.element[6+j]; 
      }
   }
//   for(int i = 0; i < 3; i++){
//      for(int j = 0; j < 3; j++){
//         result(i,j) = element[0+i] * mat(j,0) + element[3+i]*mat(j,1) + element[6+i]*mat(j,2); 
//      }
//   }
   return result;
};

double& Matrix3d::operator() (int row, int column){
   return element[row*3+column];
};

void Matrix3d::printMatrix() const {
   printf("Matrix3d:\n %f %f %f \n %f %f %f\n %f %f %f\n ", element[0], element[1], element[2], element[3], element[4], element[5], element[6], element[7], element[8]);
};

Matrix3d Matrix3d::getTransposed() const {
   Matrix3d result(element[0], element[3], element[6],
         element[1], element[4], element[7],
         element[2], element[5], element[8]);
   return result;
};

#if 0
double* Matrix3d::getEulerAngles() const {
   double* result = new double[3];

   double tr_x, tr_y;
   double C,D;
   //Code from http://skal.planet-d.net/demo/matrixfaq.htm#Q37
   result[1] = D =  asin( element[2]);        /* Calculate Y-axis angle */
   C           =  cos( result[1] );
   if ( fabs( C ) > 0.005 )             /* Gimball lock? */
   {
      tr_x      =  element[10] / C;           /* No, so get X-axis angle */
      tr_y      = -element[6]  / C;
      result[0]  = atan2( tr_y, tr_x );
      tr_x      =  element[0] / C;            /* Get Z-axis angle */
      tr_y      = -element[1] / C;
      result[2]  = atan2( tr_y, tr_x );
   }
   else                                 /* Gimball lock has occurred */
   {
      result[0]  = 0;                      /* Set X-axis angle to zero */
      tr_x      =  element[5];                 /* And calculate Z-axis angle */
      tr_y      =  element[4];
      result[2]  = atan2( tr_y, tr_x );
   }
   return result;
}
#endif
//matrix3d matrix3d::getDet(){
//    
//};

std::ostream& operator<<(std::ostream &os, const Matrix3d &m){
    return os << "[" << m.element[0] << ", " << m.element[1] <<", " << m.element[2]  <<"; " << 
       m.element[3] <<", " << m.element[4] <<", " << m.element[5] <<"; " 
       << m.element[6] <<", " << m.element[7] <<", " << m.element[8] << "]";
};

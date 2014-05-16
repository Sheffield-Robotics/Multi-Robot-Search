#ifndef VIEWER_STRUCTS_H
#define VIEWER_STRUCTS_H

#define PC_RED V_Color(1,0,0)
#define PC_YELLOW V_Color(1,1,0)
#define PC_ORANGE V_Color(0,1,1)
#define PC_GREEN V_Color(0,1,0)
#define PC_BLUE V_Color(0,0,1)
#define PC_BLACK V_Color(0,0,0)
#define PC_PURPLE V_Color(1,0,1)
#define PC_DEFAULT PC_BLACK

#include <deque>
#include <math.h>

using std::deque;

enum {
    IDX_ODO_TRACK=1,
    IDX_ODO2_TRACK,
    IDX_SM_TRACK, 
    IDX_SIXD_TRACK, 
    IDX_SIXD_ROBOT_TRACK, 
    IDX_SIXD_INTERPOL_TRACK, 
    IDX_GPS_TRACK, 
    IDX_KALMAN_TRACK, 
};

enum {
    IDX_ODO_PC=1,
    IDX_SM_PC, 
};

struct V_Color 
{
   V_Color() : r(0),g(0),b(0),a(1) {};
   V_Color(double r, double g, double b, double a=1.0) : 
        r(r), g(g), b(b), a(a)
   {}
   
   double r,g,b,a;
};
 
struct V_Vertex 
{
   V_Vertex() : x(0),y(0),z(0) {};
   V_Vertex(double x, double y, double z) : 
        x(x), y(y), z(z)
   {}
   
   V_Vertex sub(const V_Vertex& b) {
      V_Vertex result;
      result.x = x - b.x;
      result.y = y - b.y;
      result.z = z - b.z;
      return result;
   }
   V_Vertex crossProduct(const V_Vertex& b) {
      V_Vertex result;
      result.x = z * b.y - y * b.z;
      result.y = x * b.z - z * b.x;
      result.z = y * b.x - x * b.y;
      double r = sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
      result.x /= r;
      result.y /= r;
      result.z /= r;
      return result;
   }
   double x,y,z;
};

struct V_Pose3D 
{
   V_Pose3D() : x(0),y(0),z(0),yaw(0),roll(0),pitch(0),timestamp(0) {};
   V_Pose3D(double x, double y, double z) : x(x),y(y),z(z),yaw(0),roll(0),pitch(0),timestamp(0) {};

   V_Pose3D(double x, double y, double z, double yaw, double roll, double pitch) :
        x(x), y(y), z(z), yaw(yaw), roll(roll), pitch(pitch), timestamp(0) {};
   void setTime(double ts) {timestamp=ts;}
   double x,y,z,yaw,roll,pitch;
   double timestamp;
};

struct V_Face {
   unsigned int c1,c2,c3;
};

struct V_Object {
   deque<V_Vertex> verts;
   deque<V_Vertex> normals; 
   deque<V_Face> faces;
   V_Color color;
};



#endif 

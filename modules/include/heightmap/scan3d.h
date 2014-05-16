#ifndef SCAN3D_H
#define SCAN3D_H

#include "utilities/math/vector3d.h"
#include "utilities/math/matrix3d.h"
#include "utilities/misc.h"
#include <vector>
#include <string>

using std::vector;
using std::string;

/// Handles 3D Point Clouds that can be rotated and offset
class Scan3D
{
   public:
      Scan3D();
      Scan3D(const struct timeval & time);
      ~Scan3D();

      void setTime(const struct timeval & time);
      struct timeval getTime() const;

      void clearPoints();
      void addPoint(const Vector3d & v);

      /// Add the points of scan to this scan.
      void addScan(const Scan3D & scan);

      void createDebug3DScan();
      //void createFromHeights(const rescue_heightmap_update_message & msg);

      void offset(double dx, double dy, double dz);
      void rotatePitch(double angle);
      void rotateRoll(double angle);
      void rotateYaw(double angle);

      void filterMaxHeight(double max);

      const vector<Vector3d> & getPoints() const;

      void setRotation(double x, double y, double z) {rotX=x;rotY=y;rotZ=z;}
      void setTranslation(double x, double y, double z) {transX=x;transY=y;transZ=z;}

      //void sendScan(int id = 1);
      bool readFromRieglFile(string scan, string pose="");
      bool readFromHTSFile(string scan, string pose="");
      bool writeVRML(string fname, bool append=false);

      unsigned int size() const {return _points.size();}

   private:
      void getAngleAxis(double rx, double ry, double rz, double &angle, Vector3d &axis);

   protected:
      struct timeval _time;
      vector<Vector3d> _points;
      double transX,transY,transZ;
      double rotX,rotY,rotZ;
};

#endif


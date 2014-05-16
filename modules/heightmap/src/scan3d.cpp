#include "heightmap/scan3d.h"
#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <deque>

using namespace std;

#define MAX_RANGE (80.0)
//#define MAX_RANGE (20000.0)
// alles groesser MAX_RANGE abschneiden
#define MIN_RANGE (0.02)
// dto fuer kleiner MIN_RANGE

Scan3D::Scan3D()
{
   _time.tv_sec = 0;
   _time.tv_usec = 0;
}

Scan3D::Scan3D(const struct timeval & time)
{
   _time = time;
}

Scan3D::~Scan3D()
{
}

void Scan3D::setTime(const struct timeval & time)
{
   _time = time;
}

struct timeval Scan3D::getTime() const
{
   return _time;
}

void Scan3D::clearPoints()
{
   _points.clear();
}

void Scan3D::addPoint(const Vector3d & v)
{
   _points.push_back(v);
}

const vector<Vector3d> & Scan3D::getPoints() const
{
   return _points;
}
 
void Scan3D::addScan(const Scan3D & scan)
{
   _points.reserve(_points.size() + scan.size());
   _points.insert(_points.end(), scan.getPoints().begin(), scan.getPoints().end());
}


void Scan3D::createDebug3DScan()
{
   _points.clear();

   addPoint(Vector3d(10000,100,1000));
   addPoint(Vector3d(1000,100,-10000));
   addPoint(Vector3d(0,200,0));
}

/*void Scan3D::createFromHeights(const rescue_heightmap_update_message & msg)
{
   _points.clear();

   int size = msg.numHeights;
   _points.resize(size);

   for(int k = 0; k < size; k++) {
      _points[k].x = (msg.wx + msg.heights[k].dx) * 1000.0;
      _points[k].y = (msg.heights[k].height) * 1000.0;
      _points[k].z = - (msg.wy + msg.heights[k].dy) * 1000.0;
   }
}*/


void Scan3D::filterMaxHeight(double max)
{
   int validPoints = 0;
   for(int i = 0; i < (int)_points.size(); i++) {
      if(_points[i].y <= max)
         validPoints++;
   }

   vector<Vector3d> pp(validPoints);

   int index = 0;
   for(int i = 0; i < (int)_points.size(); i++) {
      if(_points[i].y <= max) {
         pp[index] = _points[i];
         index++;
      }
   }

   _points.swap(pp);
}

void Scan3D::offset(double dx, double dy, double dz)
{
   for(int i = 0; i < (int)_points.size(); i++) {
      _points[i].x += dx;
      _points[i].y += dy;
      _points[i].z += dz;
   }
}

void Scan3D::rotatePitch(double angle)
{
   Matrix3d rotMatrix(DEG2RAD(angle), 0, 0);   
   for(int i = 0; i < (int)_points.size(); i++) {
      _points[i] = rotMatrix * _points[i];
   }
}

void Scan3D::rotateYaw(double angle)
{
   Matrix3d rotMatrix(0, DEG2RAD(angle), 0);   
   for(int i = 0; i < (int)_points.size(); i++) {
      _points[i] = rotMatrix * _points[i];
   }
}

void Scan3D::rotateRoll(double angle)
{
   Matrix3d rotMatrix(0, 0, DEG2RAD(angle)); 
   for(int i = 0; i < (int)_points.size(); i++) {
      _points[i] = rotMatrix * _points[i];
   }
}

bool Scan3D::readFromRieglFile(string scan, string pose)
{
   (void) pose;
   ifstream ifs(scan.c_str(), ios::in);

   if (!ifs.is_open()) {
      cerr << "Error: cannot parse data from " << scan << endl;
      return false;
   }

   istringstream inStream;
   string line;
   getline (ifs, line);
   int num_pts = atoi(line.c_str());

   printf("Parsing %d points from file %s\n",num_pts, scan.c_str());
   clearPoints();
   int cnt = 0;
   while (!ifs.eof())  
   {
      string line;
      getline (ifs, line);
      if (line.size()<3)
         break;
      Vector3d vec;
      inStream.clear();
      inStream.str(line);
      inStream >> vec.x;
      inStream >> vec.y;
      inStream >> vec.z;
      addPoint(vec);
      if (cnt % 1000 == 1)
         printf(".");fflush(stdout);
      cnt++;
   }
   printf("Found %d points in file\n",cnt);
   return true;
}

bool Scan3D::readFromHTSFile(string scan, string pose)
{
   ifstream ifs(scan.c_str(), ios::in);

   if (!ifs.is_open()) {
      cerr << "Error: cannot parse data from " << scan << endl;
      return false;
   }

   // Try to read pose
   ifstream ifsPose(pose.c_str(), ios::in);
   if (ifsPose.is_open()) {
      istringstream inStream;
      string line;
      getline (ifsPose, line);
      inStream.clear();
      inStream.str(line);
      inStream >> transX;
      inStream >> transY;
      inStream >> transZ;
      getline (ifsPose, line);
      inStream.clear();
      inStream.str(line);
      inStream >> rotX;
      inStream >> rotY;
      inStream >> rotZ;
      ifsPose.close();
      printf("Read pose (%1.2lf, %1.2lf, %1.2lf) (%1.2lf, %1.2lf, %1.2lf)\n",transX, transY, transZ, rotX, rotY, rotZ);
   }

   istringstream inStream;
   string line;
   getline (ifs, line);

   printf("Parsing points from file %s\n",scan.c_str());
   clearPoints();
   int cnt = 0;
   while (!ifs.eof())  
   {
      string line;
      getline (ifs, line);
      if (line.size()<3)
         break;
      Vector3d vec;
      inStream.clear();
      inStream.str(line);
      inStream >> vec.x;
      inStream >> vec.y;
      inStream >> vec.z;
      addPoint(vec);
      if (cnt % 1000 == 1)
         printf(".");fflush(stdout);
      cnt++;
   }
   ifs.close();
   printf("Found %d points in file\n",cnt);
   return true;
}

void Scan3D::getAngleAxis(double phi, double theta, double psi, double &angle, Vector3d &axis) 
{
   double s;
   double a=0,b=0,c=0,d=0;
   double c1, c2, c3;
   double s1, s2, s3;

   c1 = cos(phi / 2.);
   c2 = cos(theta   / 2.);
   c3 = cos(psi   / 2.);

   s1 = sin(phi / 2.);
   s2 = sin(theta   / 2.);
   s3 = sin(psi   / 2.);

   a =  c1 * c2 * c3 + s1 * s2 * s3;
   b =  s1 * c2 * c3 - c1 * s2 * s3;
   c =  c1 * s2 * c3 + s1 * c2 * s3;
   d = -s1 * s2 * c3 + c1 * c2 * s3;

   double normSqr = (a*a + b*b + c*c + d*d);
   double norm = sqrt(normSqr);

   if (a > 1) {
      angle = 2. * acos(a/norm);
      s     = sqrt(1. - SQR(a)/normSqr);
   }
   else{
      angle = 2. * acos(a);
      s     = sqrt(1. - SQR(a));
   }

   if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
      // if s close to zero then direction of axis not important
      axis[0] = 1;
      axis[1] = 0;
      axis[2] = 0;
   }
   else {
      axis[0] = b / s; // normalise axis
      axis[1] = c / s;
      axis[2] = d / s;
   }
}

bool Scan3D::writeVRML(string fname, bool append)
{
   cout << "writing file " << fname << " ... " << flush;

   double x = transX;
   double y = transY;
   double z = transZ;

   double alpha;
   Vector3d axis;
   getAngleAxis(rotX, rotY, rotZ, alpha, axis);

   std::ofstream fout;
   if (append) 
      fout.open(fname.c_str(), ios_base::app);
   else
      fout.open(fname.c_str(), ios::out);

   fout << "#VRML V2.0 utf8\n";
   fout
      << "Transform {\n"
      //<< "  rotation " << rotX << " " << rotY << " " << rotZ << " " << alpha << "\n"
      << "  rotation " << axis[0] << " " << axis[1] << " " << axis[2] << " " << alpha << "\n"
      << "  translation " << x << " " << y << " " << z << "\n"
      << "  children [\n"
      << "    Shape {\n"
      << "      appearance Appearance {\n"
      << "         material Material {\n"
      << "            emissiveColor 1.0 1.0 1.0\n"
      << "         }\n"
      << "      }\n"
      << "      geometry PointSet {\n"
      << "          coord Coordinate {\n"
      << "              point [\n";

   for(vector<Vector3d>::iterator it = _points.begin(); it != _points.end(); it++) {
      Vector3d & pt = *it;
      fout << "            " << pt[0] << " " << pt[1] << " " << pt[2] << "," << endl;
   }
   fout << "    ]\n"
        << "  }\n"
        << "}\n"
        << "}\n"
        << "    ]\n"
        << "}\n";

   printf("Wrote point cloud with %d points\n", (int) _points.size());
   return true;
}

#if 0
void Scan3D::createFromRanges(const rescue_tilted_ranges_message & msg)
{
   _points.clear();

   int numValidRanges = 0;
   for(int i = 0; i < msg.nranges; i++) {
      if(msg.ranges[i] <= MAX_RANGE && msg.ranges[i] >= MIN_RANGE)
         numValidRanges++;
   }

   _points.resize(numValidRanges);

   int index = 0;
   for(int i = 0; i < msg.nranges; i++) {
      if(msg.ranges[i] > MAX_RANGE)
         continue;
      if(msg.ranges[i] < MIN_RANGE)
         continue;

      _points[index].x = - msg.ranges[i] * sin(DEG2RAD(msg.angles[i]));
      _points[index].z = - msg.ranges[i] * cos(DEG2RAD(msg.angles[i]));
      _points[index].y = 0;    // 2D scan is planar
      index++;
   }
}

void Scan3D::createFrom3DScan(const rescue_stair_rangescan_message & msg)
{
   rescue_rangescan_message msg2;
   msg2.id = msg.id;
   msg2.nverts = msg.nverts;
   msg2.x_coords = msg.x_coords;
   msg2.y_coords = msg.y_coords;
   msg2.z_coords = msg.z_coords;
   msg2.robot = msg.robot;
   createFrom3DScan(msg2);
}

// converts from Scan3d coordinates to our default coordinates (xz plane, y up)
void Scan3D::createFrom3DScan(const rescue_rangescan_message & msg)
{
   _points.clear();

   int numValidRanges = 0;
   for(int i = 0; i < msg.nverts; i++) {
      double range_2 = msg.x_coords[i] * msg.x_coords[i] + msg.y_coords[i] * msg.y_coords[i] + msg.z_coords[i] * msg.z_coords[i];
      if(range_2 <= MAX_RANGE * MAX_RANGE && range_2 >= MIN_RANGE * MIN_RANGE)
         numValidRanges++;
   }

   _points.resize(numValidRanges);

   int index = 0;
   for(int i = 0; i < msg.nverts; i++) {
      double range_2 = msg.x_coords[i] * msg.x_coords[i] + msg.y_coords[i] * msg.y_coords[i] + msg.z_coords[i] * msg.z_coords[i];
      if(range_2 > MAX_RANGE*MAX_RANGE || range_2 < MIN_RANGE*MIN_RANGE)
         continue;

      _points[index].x = - msg.z_coords[i];
      _points[index].y = msg.y_coords[i];
      _points[index].z = msg.x_coords[i];
      index++;
   }
}

void Scan3D::sendScan(int id)
{
   rescue_rangescan_message msg;
   msg.id = id;
   msg.nverts = _points.size();
   msg.x_coords = new float[msg.nverts];
   msg.y_coords = new float[msg.nverts];
   msg.z_coords = new float[msg.nverts];
   for(int i = 0; i < (int)_points.size(); i++) {
      msg.x_coords[i] = _points[i].x;
      msg.y_coords[i] = _points[i].y;
      msg.z_coords[i] = _points[i].z;
   }
   ComPublishToRobot(RESCUE_RANGESCAN_NAME, &msg);
   //rlogNote("Sending Scan id %d\n", id);

   delete [] msg.x_coords;
   delete [] msg.y_coords;
   delete [] msg.z_coords;
}
#endif



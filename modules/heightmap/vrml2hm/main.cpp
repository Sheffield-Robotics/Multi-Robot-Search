#include <csignal>
#include <signal.h> 
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <deque>
#include <sstream>
#include <string.h>
#include "utilities/misc.h"
#include "heightmap/heightmap.h"

using std::string;
using std::deque;
using std::istream;

#define DEBUG_PARSER (1)

void writeHeightMapPPM(string filename, HeightMap* hm);

int main( int argc, char **argv ) 
{
   string inFile = "";
   double fac = 1.0;
   double resolution = 0.1;
   int z_idx = 2;
   char c;
   bool simple = false;
   int sx=100,sy=100;
   while((c = getopt(argc, argv, "S:sz:f:F:r:h")) != EOF)
   {
      switch(c)
      {
         case 'f':
            inFile = optarg;
            break;
         case 'F':
            fac = atof(optarg);
            break;
         case 'r':
            resolution = atof(optarg);
            break;
         case 'z':
            z_idx = atoi(optarg);
            break;
         case 's':
            simple = true;
            break;
         case 'S': 
            {
               string s = optarg;
               size_t p1 = s.find_first_of( "," );
               if( string::npos != p1) 
                  s.replace(s.begin()+p1,s.begin()+p1+1," ");
               stringstream ss(s);
               ss >> sx >> sy;
            }
            break;
         case 'h':
         default:
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf(" -f <filename>\n");
            printf(" -F <num> conversion factor length\n");
            printf(" -S <num>,<num> initial grid size in X,Y\n");
            printf(" -r <num> map resolution\n");
            printf(" -z <num> specify height axis\n");
            printf(" -h Prints this help\n");
            printf(" -s Is simple point cloud with each line as 'x y z'\n");
            exit(0);
      }
   }

   if (inFile != "") 
      M_INFO3("Reading from file %s\n",inFile.c_str());
   else {
      M_ERR("Please specify input file name\n");
      return 0;
   }
   ifstream ifs;
   ifs.open(inFile.c_str());
   if (!ifs.good()) {
      M_ERR("Cannot open file!\n");
      return 0;
   }

   char *pt = getenv("PURSUIT_EVASION");
   if (pt != NULL) {
      string confFile;
      confFile = pt;
      confFile += "/config/params.ini";
      Params::readConfFile(confFile.c_str());
      M_INFO1("Read configuration from %s\n",confFile.c_str());
   }
   else
      M_ERR("Using default parameters! Did you set the PURSUIT_EVASION environment variable?\n");

   // Enforce no clamped rescale
   Params::g_ClampedRescale=0;

   //string::iterator end = inFile.begin() + inFile.find_first_of(".");
   string outFile = inFile;
   size_t n = outFile.find(".wrl"); 
   if (simple)
      n = outFile.find(".txt"); 
   outFile.replace(outFile.begin()+n,// start pointer
         outFile.begin()+n+4,    // end pointer
         "_heightmap.ppm");              // source


   cout << "Writing to " << outFile << endl;
   //outFile.assign(inFile.begin(), end);
   //outFile += ".ppm";


   // Initialize height map
   cout << "Using size " << sx << " " << sy << endl;
   HeightMap* hm = new HeightMap(resolution, 0, 0, sx, sy, false);

   // Parse file
   int cnt = 0;
   int state = 0;
   int transform = 0;
   double transX=0, transY=0, transZ=0, rotX=0, rotY=0, rotZ=0;
   bool offsetOn = false;
   double offsX=0, offsY=0, offsZ=0;
   //Scan3D* scan = new Scan3D();
   //deque<Scan3D*> scans;

   if (simple)
      state = 3;

   while (!ifs.eof())  
   {
      string line;
      getline (ifs, line);
      if (line.size()<3)
         continue;
      if (line.find("#") != string::npos)
         continue;

      if (state == 0 && line.find("rotation") != string::npos) {
         // Remove substring
         line.erase (line.find("rotation"),8);
         // Remove leading white spaces
         size_t p1 = line.find_first_of( " " );
         size_t p2 = line.find_first_not_of( " " ); 
         if( string::npos != p1   && string::npos != p2 && p1<p2) 
            line.erase(p1,p2-p1);
         istringstream inStream(line);
         double axis[3];
         double alpha=0.0;
         inStream >> axis[0];
         inStream >> axis[1];
         inStream >> axis[2];
         inStream >> alpha;
         if (DEBUG_PARSER) printf("Rotation alpha %lf axis (%lf  %lf %lf)\n",alpha, axis[0], axis[1], axis[2]);
         double normSqr = 0;
         for (int i=0; i<3; ++i) normSqr += axis[i]*axis[i];
         double axis_norm = sqrt(normSqr);
         double a,b,c,d;
         a = cos(alpha/2.);
         b = axis[0] * sin(alpha/2.) / axis_norm; 
         c = axis[1] * sin(alpha/2.) / axis_norm;
         d = axis[2] * sin(alpha/2.) / axis_norm;
         rotZ = atan2(2.0*(a*d + b*c), (1 - 2*(c*c + d*d)));
         double m20 = -2.0 * (b*d - c*a);
         double m21 = 2.0 * (c*d + b*a);
         double m22 = a*a - b*b - c*c + d*d;
         rotY = atan2(m20, sqrt(m21*m21 + m22*m22));
         rotX = atan2(2.0*(c*d + b*a), a*a - b*b - c*c + d*d);
         transform++;
      }
      if (state == 0 && line.find("translation") != string::npos) {
               // Remove substring
         line.erase (line.find("translation"),11);
               // Remove leading white spaces
         size_t p1 = line.find_first_of( " " );
         size_t p2 = line.find_first_not_of( " " );
         if( string::npos != p1   && string::npos != p2 && p1<p2)
            line.erase(p1,p2-p1);

         istringstream inStream(line);
         inStream >> transX;inStream >> transY;inStream >> transZ;
         if (DEBUG_PARSER) printf("Translation (%lf  %lf %lf)\n",transX, transY, transZ);
         transform++;
      }
      // Add transform to scan
      if (transform >= 2) {
         //scan->setTranslation(transX,transY,transZ);
         //scan->setRotation(rotX,rotY,rotZ);
         transform = 3;
      }
      if (state == 0 && line.find("PointSet") != string::npos)
         state = 1;
      if (state == 1 && line.find("Coordinate") != string::npos)
         state = 2;
      if (state == 2 && line.find("point") != string::npos) {
         state = 3;
         continue;
      }
      // Scan completed
      if (state == 3 && line.find("]") != string::npos) {
         //scans.push_back(scan);
         //if (DEBUG_PARSER) M_INFO1("Added scan with %d points\n",(int) scan->size()); 
         //scan = new Scan3D();
         if (DEBUG_PARSER) M_INFO1("Added scan with %d points\n",(int) cnt); 
         state = 0;
         break;
      }
      // Parse line with point
      if (state == 3 ) {
         // Replace all comma
         string::size_type pos = 0;
         while ( ( pos = line.find (",",pos) ) != string::npos )
         {
            line.replace ( pos, 1," ");
         }

         istringstream inStream(line);
         double point[3];
         inStream >> point[0];inStream >> point[1];inStream >> point[2];
         //scan->addPoint(Vector3d(point[0]*1000.0*fac,point[1]*1000.0*fac,point[2]*1000.0*fac));
         double z = point[z_idx];
         double x=0,y=0;
         if (z_idx==0) {
            x = point[1];y = point[2];
         }
         else if (z_idx==1) {
            x = point[0];y = point[2];
         }
         else if (z_idx==2) {
            x = point[0];y = point[1];
         }
         else {
            printf("Wrong z index\n");
            exit(0);
         } 

         if (offsetOn == false) {
            offsX = x;
            offsY = y;
            offsZ = z;
            offsetOn = true;
            printf("Substracting offset (%1.2lf %1.2lf %1.2lf)\n",offsX,offsY,offsZ);
         }

         x -= offsX;
         y -= offsY;
         z -= offsZ;

         hm->updateFromPointMM(Vector3d(x*fac, y*fac, z*fac));
      }

      if (cnt % 1000 == 1)
         printf(".");fflush(stdout);
      cnt++;
   }
   
   //if (DEBUG_PARSER) M_INFO1("Found %d scans\n",(int) scans.size()); 

   // Update height map
   //for (unsigned int i=0; i<scans.size(); i++) 
   //   hm->updateFromWorldScan(*scans[i]);

   // Write height map ppm
   writeHeightMapPPM(outFile.c_str(),hm);

   // Clean-up
   //for (unsigned int i=0; i<scans.size(); i++) 
   //   delete scans[i];
   delete hm;
   M_INFO1("Completed.\n" );
   return 0;
}


void writeHeightMapPPM(string filename, HeightMap* hm)
{
   int sx = hm->sizeX();
   int sy = hm->sizeZ();

   M_INFO3("Saving HM PPM of size %d,%d to %s\n",sx, sy, filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P5" << endl;
   os << sy << " " << sx << " " << 255 << endl;

   // Find max
   double max = 0.0;
   double min = HUGE_VAL;
   for(int x=0; x<sx; x++) {
      for(int y=0; y<sy; y++) {
         double value = hm->getCellsMM()[x][y].getHeight();
         if (value>max)
            max=value;
         if (value<min)
            min=value;
      }
   }
   //M_DEBUG("MAX Height IS %lf\n",max);
   //M_DEBUG("MIN Height IS %lf\n",min);
   for(int x=0; x<sx; x++) {
      for(int y=0; y<sy; y++) {
         double value = hm->getCellsMM()[x][y].getHeight();
         double f = 255.0 * ((value - min) / (max-min));
         char c = (unsigned char) f;
         os.put(c);
      }
   }
   os.close();
}

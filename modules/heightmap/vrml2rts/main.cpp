#include <csignal>
#include <signal.h> 
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <sys/time.h>
#include <iomanip>

using namespace std;

int main( int argc, char **argv ) 
{
   string inFile = "";
   int num =0;

   char c;
   while((c = getopt(argc, argv, "n:f:h")) != EOF)
   {
      switch(c)
      {
         case 'f':
            inFile = optarg;
            break;
         case 'n':
            num = atoi(optarg);
            break;
         case 'h':
         default:
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf(" -f <filename>\n");
            printf(" -n <num> number of file to write\n");
            printf(" -h Prints this help\n");
            exit(0);
      }
   }

   if (inFile == "")  {
      fprintf(stderr,"Please specify input file\n");
      return 0;
   }
   ifstream in;
   in.open(inFile.c_str());
   if (!in.is_open()) {
      fprintf(stderr,"Error: cannot open input file.\n");
      return 0;
   }

   //string::iterator s1 = inFile.begin() + inFile.find_first_of("_") + 1;
   //string::iterator s2 = inFile.begin() + inFile.find_first_of(".");
   //string number;
   //number.assign(s1,s2);
   //int n = atoi(number.c_str());
   //cout << n << endl;

   char dummy[100];
   sprintf(dummy, "scan3d_0_%03d.3d",num);
   string scanF(dummy);

   cout << "Infile: " << inFile << endl;
   cout << "Outfile: " << scanF << endl;

   //pcloud_0000343.wrl
   //scan3d_0_390.3d

   string poseF = "odometry_0_sync_interpol.dat";
   ofstream outP, outS;
   outP.open(poseF.c_str(), std::ios_base::app);
   outS.open(scanF.c_str());

   istringstream inStream;
   string line;

   int cnt = 0;
   int state = 0;
   int transform = 0;
   double transX=0, transY=0, transZ=0, rotX=0, rotY=0, rotZ=0;
   while (!in.eof())  
   {
      string line;
      getline (in, line);
      if (line.size()<3)
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
         //printf("Rotation alpha %lf axis (%lf  %lf %lf)\n",alpha, axis[0], axis[1], axis[2]);
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
         inStream >> transX;
         inStream >> transY;
         inStream >> transZ;
         //printf("Translation (%lf  %lf %lf)\n",transX, transY, transZ);
         transform++;
      }
      if (transform >= 2) {
         //struct timeval tv;
         //gettimeofday(0,&tv);
         // write out transformation
         //outP << ((double) tv.tv_sec + (double) tv.tv_usec/1e6) << " ";
         outP <<  setiosflags(ios::fixed) << scientific <<setprecision(7);
         outP <<  0.0 << " ";
         outP << transX << " " << transY << " " << transZ << " ";
         outP << rotX << " " << rotY << " " << rotZ << "\n";
         outP.close();
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
      if (state == 3 && line.find("]") != string::npos) {
         state = 4;
         break;
      }
      if (state == 3 ) {
         outS << line << endl;
      }

      if (cnt % 1000 == 1)
         printf(".");fflush(stdout);
      cnt++;
   }

   outP.close();
   outS.close();

   printf("Found %d points\n",cnt);
   return 1;
}


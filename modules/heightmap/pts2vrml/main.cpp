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

const double scale = 100.0;

int main( int argc, char **argv ) 
{
   string inFile = "";
   double maxrange = -1.0;

   char c;
   while((c = getopt(argc, argv, "f:hm:")) != EOF)
   {
      switch(c)
      {
         case 'f':
            inFile = optarg;
            break;
         case 'm':
            maxrange = atof(optarg);
            break;
         case 'h':
         default:
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf(" -f <filename>\n");
            printf(" -m <num> max range (default: %lf)\n",maxrange);
            printf(" -h Prints this help\n");
            exit(0);
      }
   }

   if (inFile == "")  {
      fprintf(stderr,"Please specify input file\n");
      return 0;
   }
   ifstream fin;
   fin.open(inFile.c_str());
   if (!fin.is_open()) {
      fprintf(stderr,"Error: cannot open input file.\n");
      return 0;
   }

   string::iterator end = inFile.begin() + inFile.find_first_of(".");
   string outFile;
   outFile.assign(inFile.begin(), end);
   outFile += ".wrl";

   cout << "Infile: " << inFile << endl;
   cout << "Outfile: " << outFile << endl;

   ofstream fout;
   fout.open(outFile.c_str());
   fout << "#VRML V2.0 utf8\n";
   fout
//      << "Transform {\n"
//      << "  rotation " << axis[0] << " " << axis[1] << " " << axis[2] << " " << alpha << "\n"
//      << "  translation " << x << " " << y << " " << z << "\n"
      << "  Shape {\n"
      << "      appearance Appearance {\n"
      << "         material Material {\n"
      << "            emissiveColor 1.0 1.0 1.0\n"
      << "         }\n"
      << "      }\n"
      << "    geometry ";
   fout << "PointSet {\n";
   fout << "  coord Coordinate {\n";
   fout << "    point [\n";

   string line;
   // Determine offset
   int cnt = 0;
   double ox=0, oy=0, oz=0;
   while (fin.good())  
   {
      string line;
      getline (fin, line);
      if (line.size()<3)
         continue;
      if (line.find('#') != string::npos)
         continue;

      // Replace all comma
      string::size_type pos = 0;
      while ( ( pos = line.find (",",pos) ) != string::npos )
      {
         line.replace ( pos, 1," ");
      }
      istringstream inStream(line);
      double point[3];

      inStream >> point[0] >> point[1] >> point[2];
      ox += point[0]; 
      oy += point[1]; 
      oz += point[2]; 
      cnt++;
   }
   ox /= cnt; oy /= cnt; oz /= cnt;
   printf("Using offset: %1.2lf %1.2lf %1.2lf\n",ox,oy,oz);
   fin.close();
   fin.open(inFile.c_str());

   while (fin.good())  
   {
      string line;
      getline (fin, line);
      if (line.size()<3)
         continue;
      if (line.find('#') != string::npos)
         continue;

      // Replace all comma
      string::size_type pos = 0;
      while ( ( pos = line.find (",",pos) ) != string::npos )
      {
         line.replace ( pos, 1," ");
      }

      //cout << line << endl;
      istringstream inStream(line);
      double point[3];
      inStream >> point[0] >> point[1] >> point[2];
      //point[0]-=ox;
      //point[1]-=oy;
      //point[2]-=oz;
      point[0] /= scale;
      point[1] /= scale;
      point[2] /= scale;
 
      if (maxrange>0.0) {
         double r = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);
         if (r<=maxrange)
            fout << point[0] << point[1] << point[2] << endl;
      }
      else
         fout << point[0] << " " << point[1] << " " << point[2] << endl;
      if (cnt % 10000 == 1)
         printf(".");fflush(stdout);
      cnt++;
      //if (cnt>10)
       //  break;
   }
   fout << "    ]\n";
   fout << "  }\n";
   fout << "}\n";
   fout << "}\n";
   fout.close();
   fin.close();

   printf("Processed point cloud with %d points\n", cnt);
   return 1;
}


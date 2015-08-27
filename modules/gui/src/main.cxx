#include <qapplication.h>
#include <time.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "heightmap/heightmap.h"
#include "gui/viewer.h"
#include "gui/animator.h"
#include "utilities/filesysTools.h"
#include "utilities/paramfile.h"
#include "heightmap/visibility.h"
#include "heightmap/perimeter.h"
#include "heightmap/geomap.h"
#include <sstream>

#include<QTextStream>
#include<QFile>

#define INITIAL_SIZE_X  100 // in cells
#define INITIAL_SIZE_Y  100 // in cells

static void quitproc(int signal);
bool quit_signal = 0;
bool redraw = true;
string confFile = "";

extern int _rasterWidth;
extern int _rasterHeight;
extern uint32 *_rasterImage;

Viewer *viewer = NULL;

void allocVisibility(Visibility** vis, HeightMap** map, string tiffFilename, bool openExistingGraph);
void allocAnimator(Animator** anim, Visibility** vis, HeightMap** map, string tiffFilename, bool openExistingSchedule);
void allocPerimeter(Perimeter** per, Visibility** vis, HeightMap** map, string tiffFilename, bool openExistingGraph);

int main(int argc, char** argv)
{
   char c;
   string tiffFilename ="";
   string imageTiffFilename ="";
   bool openExistingSchedule = false;
   bool openExistingGraph = false;
   bool useGui = true;
   string gpsPort = "";
   string gpsFile = "";
   string inputLogFile = "";
   string inputLogFilelPlay = "";
   bool playGpsLog = false;

   // Initialize parameters from file

   char *pt = getenv("PURSUIT_EVASION");
   if (pt != NULL) {
      confFile = pt;
      confFile += "/config/params.ini";
      Params::readConfFile(confFile.c_str());
      M_INFO1("Read configuration from %s\n",confFile.c_str());
   }
   else
      M_ERR("Using default parameters! Did you set the PURSUIT_EVASION environment variable?\n");

   while((c = getopt(argc, argv, "f:r:p:t:es:bg:d:l:i:PS:hH12REXUYD")) != EOF)
   {
      switch(c)
      {
         case 'f':
            tiffFilename = optarg;
            break;
         case 'i':
            imageTiffFilename = optarg;
            break;
        case 'r':
            Params::g_pursuer_range = atof(optarg);
            break;
         case 'p':
            Params::g_pursuer_height = atof(optarg);
            break;
         case 't':
            Params::g_target_height = atof(optarg);
            break;
         case 'e':
            Params::g_run_experiment_mode = 1;
            break;
         case 's':
            Params::g_num_spanning_trees = atoi(optarg);
            break;
         case 'R':
            Params::g_use_improved_sampling = 0;
            break;
         case 'b':
            Params::g_bias_spanning_trees = true;
            break;
         case 'g':
            gpsPort = optarg;
            break;
         case 'l':
            inputLogFile = optarg;
            break;
         case 'P':
            playGpsLog = true;
            break;
         case 'S':
            Params::g_shrub_height = atof(optarg);
            break;
         case '1':
            openExistingGraph = true;
            break;
         case '2':
            openExistingGraph = true;
            openExistingSchedule = true;
            break;
         case 'E':
            Params::g_generate_regular_instead_of_sparse_edges = 1;
            break;
         case 'Y':
            Params::g_compute_shady_edges = 1;
            break;
         case 'X':
            Params::g_compute_exhaustive_spanning_trees = 1;
            break;
         default:
         case 'h':
         case 'H':
            printf("\nOptions:\n");
            printf("--------------\n");
            printf("-f <filename> Load height map from GeoTIFF file.\n");
            printf("-i <filename> Load image map from GeoTIFF file.\n");
            printf("-1 open existing graph file with same name as map \n");
            printf("-2 open existing graph and schedule file with same name as map \n");
            printf("-r <num> pursuer range (default is %1.2lf m)\n",Params::g_pursuer_range);
            printf("-p <num> pursuer height (default is %1.2lf m)\n",Params::g_pursuer_height);
            printf("-t <num> target height (default is %1.2lf m)\n",Params::g_target_height);
            printf("-e run experiment mode (without GUI)\n");
            printf("-s number of spanning trees\n");
            printf("-b bias spanning trees\n");
            printf("-g <port> use GPS receiver at port\n");
            printf("-l <filename> display GPS logfile\n");
            printf("-P play/simulate GPS logfile\n");
            printf("-R use random sampling instead of improved sampling\n");
            printf("-E use regular instead of sparse edges (see journal)\n");
            printf("-X compute exhaustive spanning trees (enumerate all of them)\n");
            printf("-Y prune shady edges\n");
            printf("-R use random sampling\n");
            printf("-S <num> shrub height (default is %1.2lf m)\n",Params::g_shrub_height);
            printf("\n");
            exit(0);
            break;
      }
   }

   if (tiffFilename == "") {
      M_ERR("You need to provide a tiff file!\n");
      exit(0);
   }

   if (Params::g_run_experiment_mode)
      useGui=false;

   M_INFO2("Pursuer height=%1.2lf, pursuer range=%1.2lf, Target height=%1.2lf\n",Params::g_pursuer_height, Params::g_pursuer_range, Params::g_target_height);

   // Initialize randon generator
   srand48(time(NULL));

   // Initialize signal handler
   signal(SIGINT, quitproc);

   // Initialize Heightmapper
   HeightMap* map = new HeightMap(0.1, 0, 0, INITIAL_SIZE_X, INITIAL_SIZE_Y);


      // Read command lines arguments.
      QApplication *application = NULL;
      
      Animator* anim = NULL;
      Visibility* vis = NULL;
      Perimeter* per = NULL;

      bool ret = true;
      GeoMap* geo = NULL;
      if (useGui) {
         // Prepare Gui
         M_INFO1("Starting viewer ...\n");
         application = new QApplication(argc,argv);
         // Instantiate the viewer.
         viewer = new Viewer();
         viewer->setPointers(map,NULL,NULL,NULL);
         viewer->setWindowTitle("Terrain Viewer 3d");

         // Load Height Map
         if (tiffFilename != "")
            ret = viewer->loadHeightmapFromTIFF(tiffFilename);
         else 
            ret = viewer->loadHeightmapFromTIFF("");
         if (imageTiffFilename != "") {
            ret = viewer->loadImageMapFromTIFF(imageTiffFilename);
            if (ret) M_INFO3("Loaded image map from %s\n",imageTiffFilename.c_str());
         }
         // Allocate animator, visibility and perimeter
         allocVisibility(&vis, &map, tiffFilename, openExistingGraph);
         allocPerimeter(&per, &vis, &map, tiffFilename, openExistingGraph);
         allocAnimator(&anim, &vis, &map, tiffFilename, openExistingSchedule);
         viewer->setPointers(map,vis,anim,per);

         // Make the viewer window visible on screen.
         viewer->show();
      }
      else if (!useGui && tiffFilename != "") 
      {
         geo = new GeoMap();
         ret = geo->loadGeoTiff(tiffFilename);
         if(ret) {
            uint32* dataBuf;
            geo->getData(dataBuf); 

            // Handle map masking
            uint32* maskDataBuf = NULL;
            string maskFile = getPureFilename(tiffFilename);
            maskFile += "_mask.tiff";
            GeoMap* mask = new GeoMap();
            if (fileExists(maskFile) && mask->loadGeoTiff(maskFile)) 
            {
               if (mask->width() != geo->width() || mask->height() != geo->height() || 
                   mask->resolution() != geo->resolution()) {
                  M_ERR("Mask file %s is not fitting for height map %s\n",maskFile.c_str(), tiffFilename.c_str());
               }
               else {
                  mask->getData(maskDataBuf);
                  M_INFO3("Read mask data from %s for offline process\n",maskFile.c_str());
               }
            }
            else {
               M_INFO3("Cannot find mask file, activating auto classifier!\n");
               Params::g_classify_from_mask_only = 0;
            }

            // Load Height Map
            ret = map->loadFromTIFF(dataBuf, geo->width(), geo->height(), geo->resolution(), geo->heightResolution(), false, maskDataBuf);

            // Allocate animator, visibility and perimeter
            allocVisibility(&vis, &map, tiffFilename, openExistingGraph);
            allocPerimeter(&per, &vis, &map, tiffFilename, openExistingSchedule);
            allocAnimator(&anim, &vis, &map, tiffFilename, openExistingSchedule);
    
            delete mask;
         }
      }

      if (!ret) {
         M_ERR("Something went wrong while loading file\n");
         if(map != NULL) delete map;
         if(vis != NULL) delete vis;
         if(per != NULL) delete per;
         if(anim != NULL) delete anim;
         if (viewer != NULL) delete viewer;
         M_INFO1("Bye ...\n");
         exit(0);
      }






      // MAIN LOOP
      while (!quit_signal) 
      {
         // Update viewer
         if (useGui) {
            if (!viewer->isVisible())
               quit_signal = true;
            if (redraw) {
               viewer->updateGL();
               redraw = false;
            }
            application->processEvents();
            usleep(1000);

         }
   }
   delete map;
   delete vis;
   delete per;
   delete anim;
   delete viewer;
   M_INFO1("Bye ...\n");
}


void allocVisibility(Visibility** vis, HeightMap** map, string tiffFilename, bool openExistingGraph)
{
   if (*map == NULL) {
      M_ERR("Got empty pointer\n");
      return;
   }
 
   if (*vis != NULL)
      delete *vis;

   // Generate Visibility stuff
   *vis = new Visibility(*map, Params::g_pursuer_height, Params::g_pursuer_range, Params::g_target_height);
   if (openExistingGraph) {
      std::string pure = getPureFilename(tiffFilename);
      pure += ".dot";
      (*vis)->loadGraph(pure);
   }
}

void allocPerimeter(Perimeter** per, Visibility** vis, HeightMap** map, string tiffFilename, bool openExistingGraph)
{
   if (*map == NULL) {
      M_ERR("Got empty pointer\n");
      return;
   }
 
   if (*per != NULL)
      delete *per;

   // Generate Visibility stuff
   *per = new Perimeter(*map, *vis, Params::g_pursuer_height, Params::g_pursuer_range, Params::g_target_height);
   if (openExistingGraph) {
      std::string pure = getPureFilename(tiffFilename);
      pure += ".peri";
      (*per)->load(pure);
   }
}



void allocAnimator(Animator** anim, Visibility** vis, HeightMap** map, string tiffFilename, bool openExistingSchedule)
{
   if (*vis == NULL || *map == NULL) {
      M_ERR("Got empty pointer\n");
      return;
   }
   if (*anim != NULL)
      delete *anim;

   // Generate Animator
   *anim = new Animator(*vis, *map);
   if (openExistingSchedule) {
      if (!Params::g_use_compressed_strategy) {
         string pure = getPureFilename(tiffFilename);
         pure += ".sch";
         (*anim)->init(pure, false);
      }
      else {
         string pure = getPureFilename(tiffFilename);
         pure += ".cst";
         (*anim)->init(pure, true);
      }
   }
}


void quitproc(int signal)
{
   (void)signal;
   quit_signal = 1;
}



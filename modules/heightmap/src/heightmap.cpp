#include "heightmap/heightmap.h"
#include "heightmap/scan3d.h"
#include "utilities/timeutil.h"
#include "utilities/timing.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "utilities/misc.h"
#include "agents/agentstate.h"

#define CONTINUOUSLY_UPDATE_VARIANCES (0)

//#define BEAM_VARIANCE (100.0 * 100.0) // in mm
#define BEAM_VARIANCE (10.0 * 10.0) // in mm

#define DO_NOT_SMOOTH_VARIANCES (1)
///< If true use non smoothed variances in smoothed map to ensure that the range of variances will be the same in the non-smoothed and in the smoothed map.

#define MAX_SMOOTH_MAHALNOBIS_DIST (2)
///< Max Mahalanobis distance that a cell is considered a neighbour to; Set to 0 for smooth all cells.

#define SMOOTH_DO_MEDIAN (1)

//#define PRINT_CONFUSION_MATRIX_ON_SET
//#define DO_HEIGHTMAP_TIMING

#define HEIGHTMAP_VERSION "HEIGHTMAP_V1.0"

using namespace std;

HeightMap::HeightMap(double resolution, double worldOffsX, double worldOffsZ, int sizeX, int sizeZ, bool mrfWeightsLoadDebug) 
{
   _id = 0;
   _sizeX = sizeX;
   _sizeZ = sizeZ;

   // Convert to mm
   _worldOffsetX = worldOffsX * 1000.0;
   _worldOffsetZ = worldOffsZ * 1000.0;

   // Convert to mm
   _resolution = resolution * 1000.0;
   _heightResolution = resolution * 1000.0;

   if((_sizeX == 0) || (_sizeZ == 0))
      _cells = NULL;
   else
      _cells = createNewMap(_sizeX, _sizeZ);

   _lastRescaleRobotX = HUGE_VAL;      ///< invalid value to force rescale enabled.
   _lastRescaleRobotZ = HUGE_VAL;

   _sizeSmoothedX = 0;
   _sizeSmoothedZ = 0;
   _smoothedCells = NULL;

   _smoothingEnabled = false;
   _lastUpdateTime.tv_sec = 0;
   _lastUpdateTime.tv_usec = 0;

   _classifier = new HeightMapClassifier(mrfWeightsLoadDebug);
   _classifyCompleteUpdateAsWall = true;
}

HeightMapClassifier* HeightMap::getClassifier()
{
   return _classifier;
}

void HeightMap::setSmoothing(bool enable)
{
   _smoothingEnabled = enable;
   if(_smoothingEnabled) {
      M_INFO1("Map Smoothing enabled.\n");
   } else {
      M_INFO1("Map Smoothing disabled.\n");
   }
};

/** Never call this function, if you don't know what you're doing ! */
void HeightMap::deleteSmoothedMap()
{
   if(_smoothedCells == NULL)
      return;

   for(int x = 0; x < _sizeX; x++) {
      delete [] _smoothedCells[x];
   }
   delete [] _smoothedCells;
   _smoothedCells = NULL;

   _sizeSmoothedX = 0;
   _sizeSmoothedZ = 0;
}

HeightMap::~HeightMap()
{
   if(_cells != NULL) {
      for(int i = 0; i < _sizeX; i++) {
         delete [] _cells[i];
      }
      delete [] _cells;
   }

   if(_smoothedCells != NULL) {
      for(int i = 0; i < _sizeSmoothedX; i++) {
         delete [] _smoothedCells[i];
      }
      delete [] _smoothedCells;
   }

   delete _classifier;
}

HeightCell** HeightMap::createNewMap(int sx, int sz) const
{
   HeightCell** map = new HeightCell*[sx];
   for(int i = 0; i < sx; i++) {
      map[i] = new HeightCell[sz];
   }
   //M_INFO2("Created Map of size x=%d z=%d\n",sx,sz);
   return map;
}


void HeightMap::updateFromWorldScan(const Scan3D & scan)
{
   _lastUpdateTime = scan.getTime();
   int num = scan.size();
   const vector<Vector3d> & points = scan.getPoints();

   // Determine bounding box
   double min_x = HUGE_VAL;
   double max_x = - HUGE_VAL;
   double min_y = HUGE_VAL;
   double max_y = - HUGE_VAL;
   double min_z = HUGE_VAL;
   double max_z = - HUGE_VAL;

   for(int i = 0; i < num; i++) 
   {
      if(points[i].x > max_x) max_x = points[i].x;
      if(points[i].x < min_x) min_x = points[i].x;
      if(points[i].y > max_y) max_y = points[i].y;
      if(points[i].y < min_y) min_y = points[i].y;
      if(points[i].z > max_z) max_z = points[i].z;
      if(points[i].z < min_z) min_z = points[i].z;
   }

   printf("Size of area: X (%1.2lf ... %1.2lf m) Y(%1.2lf ... %1.2lf m)  Z(%1.2lf ... %1.2lf m)\n",
         min_x/1000.0,max_x/1000.0,min_y/1000.0,max_y/1000.0,min_z/1000.0,max_z/1000.0);

   if(CONTINUOUSLY_UPDATE_VARIANCES) {
      for(int x = 0; x < _sizeX; x++) {
         for(int y = 0; y < _sizeZ; y++) {
            _cells[x][y].updateErrorVariance(scan.getTime());
         }
      }
   }

   if(Params::g_ClampedRescale)
      clampedRescaleMap(min_x, max_x, min_y, max_y);
   else
      rescaleMap(min_x, max_x, min_y, max_y);

   int gx, gy;
   for(int i = 0; i < num; i++) 
   {
      world2gridMM(points[i].x, points[i].y, gx, gy);
      if(gx < 0 || gy < 0 || gx >= _sizeX || gy >= _sizeZ) {
         if(!Params::g_ClampedRescale)    // with clamped rescale, this can happen.
            M_WARN("%s: Trying to access cell (%d, %d) in a %d x %d map. This should never happen.\n", __func__, gx, gy, _sizeX, _sizeZ);
         continue;
      }
      //printf("Integrating %d %d\n",gx,gy);
      _cells[gx][gy].integrateMeasurement(points[i].z, BEAM_VARIANCE, scan.getTime());
   }
}

void HeightMap::updateFromPointMM(const Vector3d& pp)
{
   // Determine bounding box
   static double min_x = HUGE_VAL;
   static double max_x = - HUGE_VAL;
   static double min_y = HUGE_VAL;
   static double max_y = - HUGE_VAL;
   //static double min_z = HUGE_VAL;
   //static double max_z = - HUGE_VAL;

   // Convert to MM
   double fak = 1000.0;
   Vector3d p(pp.x*fak, pp.y*fak, pp.z*fak);

   bool up=false;
   if(p.x > max_x) {max_x = p.x;up=true;}
   if(p.x < min_x) {min_x = p.x;up=true;}
   if(p.y > max_y) {max_y = p.y;up=true;}
   if(p.y < min_y) {min_y = p.y;up=true;}
   //if(p.z > max_z) {max_z = p.z;up=true;}
   //if(p.z < min_z) {min_z = p.z;up=true;}

   if (up) {
      //printf("New size of area: X (%1.2lf ... %1.2lf m) Y(%1.2lf ... %1.2lf m)  Z(%1.2lf ... %1.2lf m)\n",
      //   min_x/1000.0 ,max_x/1000.0 ,min_y/1000.0 ,max_y/1000.0 ,min_z/1000.0 ,max_z/1000.0);
      if(Params::g_ClampedRescale)
         clampedRescaleMap(min_x, max_x, min_y, max_y);
      else
         rescaleMap(min_x, max_x, min_y, max_y);
   }

   int gx, gy;
   world2gridMM(p.x, p.y, gx, gy);
   if(gx < 0 || gy < 0 || gx >= _sizeX || gy >= _sizeZ) {
      M_WARN("%s: Trying to access cell (%d, %d) in a %d x %d map.\n"
               , __func__, gx, gy, _sizeX, _sizeZ);
      return;
   }
   struct timeval tv;
   _cells[gx][gy].integrateMeasurement(p.z, BEAM_VARIANCE, tv);
}

HeightCell** HeightMap::getCellsMM() 
{
   return _cells;
}

HeightCell** HeightMap::getSmoothedCellsMM() 
{
   return _smoothedCells;
}

/**
 * Rescale to accomodate for a scan ranging in [min_x, max_x], [min_z, max_z].
 *
 * Rescaling will align the map to have the robot in its center.
 *
 * Rescaling will be issued, if the scan does not fit into the map
 * and the robot has moved substancially (e.g. min 1m)
 */
void HeightMap::clampedRescaleMap(double min_x, double max_x, double min_z, double max_z)
{
   if(_sizeX == 0 || _sizeZ == 0) {    // first update
      int cellsX = (int)(Params::g_ClampedRescaleLength*1000.0/_resolution);
      int cellsZ = (int)(Params::g_ClampedRescaleLength*1000.0/_resolution);
      double worldSizeX = cellsX * _resolution;
      double worldSizeZ = cellsZ * _resolution;
      _worldOffsetX = AgentState::robotX - worldSizeX/2.0;   // center map in world
      _worldOffsetZ = AgentState::robotY - worldSizeZ/2.0;   // center map in world
      _sizeX = cellsX;
      _sizeZ = cellsZ;
      _cells = createNewMap(_sizeX, _sizeZ);
      M_INFO1("Created HeightMap Size(%d x %d) from (%.2f, %.2f) to (%.2f, %.2f)\n", 
            _sizeX, _sizeZ, _worldOffsetX, _worldOffsetZ, (_sizeX - 1) * _resolution + _worldOffsetX, (_sizeZ - 1) * _resolution + _worldOffsetZ);
      _lastRescaleRobotX = AgentState::robotX;
      _lastRescaleRobotZ = AgentState::robotY;
      return;
   }

   // Determine if rescale necessary
   double min_world_x, max_world_x, min_world_z, max_world_z;
   grid2worldMM(min_world_x, min_world_z, 0, 0);
   grid2worldMM(max_world_x, max_world_z, _sizeX - 1, _sizeZ - 1);

   bool scanOutOfBounds_x = false;
   bool scanOutOfBounds_z = false;
   if(min_x < min_world_x || max_x > max_world_x) {
      scanOutOfBounds_x = true;
   }
   if(min_z < min_world_z || max_z > max_world_z) {
      scanOutOfBounds_z = true;
   }
   bool scanOutOfBounds = scanOutOfBounds_x || scanOutOfBounds_z;

   bool rescaleDist = false;
   const double minDistBeforeRescale = 1.0;  ///< m
   double distSinceLastRescale = hypot(_lastRescaleRobotX - AgentState::robotX, _lastRescaleRobotZ - AgentState::robotY);
   if(distSinceLastRescale > minDistBeforeRescale * 1000.0) {
      rescaleDist = true;
   }

   bool rescale = false;
   if(scanOutOfBounds && rescaleDist)
      rescale = true;

   if(!rescale)
      return;
   // now we want to rescale

   // Determine parameters of new map. 
   int newCellsX = (int)(Params::g_ClampedRescaleLength*1000.0/_resolution);
   int newCellsZ = (int)(Params::g_ClampedRescaleLength*1000.0/_resolution);
   // this is pretty nasty: some code relies on changing _sizeX to detect a rescale, so I force this
   // _sizeX will be alternating between g_ClampedRescaleLength/_resolution and g_ClampedRescaleLength/_resolution + 1 for each rescale
   if(_sizeX == newCellsX) {
      newCellsX++;
   }
   double newWorldSizeX = newCellsX * _resolution;
   double newWorldSizeZ = newCellsZ * _resolution;
   double newOffsetX = AgentState::robotX - newWorldSizeX/2.0;
   double newOffsetZ = AgentState::robotY - newWorldSizeZ/2.0;

   // Perform the actual rescale
   saveRescaleInformation(true);

   HeightCell** newMap = createNewMap(newCellsX, newCellsZ);
   // Fill new Map with the old map
   for(int x = 0; x < _sizeX; x++) {
      for(int z = 0; z < _sizeZ; z++) {
         double world_x, world_z;
         grid2worldMM(world_x, world_z, x, z);
         int new_index_x = (int) ((world_x - newOffsetX) / _resolution);
         int new_index_z = (int) ((world_z - newOffsetZ) / _resolution);
         if(new_index_x < 0 || new_index_z < 0)
            continue;
         if(new_index_x >= newCellsX || new_index_z >= newCellsZ)
            continue;

         newMap[new_index_x][new_index_z] = _cells[x][z];
      }
   }

   // delete old map
   for(int i = 0; i < _sizeX; i++) {
      delete [] _cells[i];
   }
   delete [] _cells;

   _cells = newMap;
   _sizeX = newCellsX;
   _sizeZ = newCellsZ;
   _worldOffsetX = newOffsetX;
   _worldOffsetZ = newOffsetZ;
   _lastRescaleRobotX = AgentState::robotX;
   _lastRescaleRobotZ = AgentState::robotY;

   M_INFO1("Rescaled HeightMap Size(%d x %d) from (%.2f, %.2f) to (%.2f, %.2f)\n", _sizeX, _sizeZ, _worldOffsetX, _worldOffsetZ, (_sizeX - 1) * _resolution + _worldOffsetX, (_sizeZ - 1) * _resolution + _worldOffsetZ);
   // there is/was a smoothed map, also resize
   if(_smoothedCells != NULL) {
      M_INFO1("Rescaling smoothed map to map size.\n");
      for(int i = 0; i < _sizeSmoothedX; i++) {
         delete [] _smoothedCells[i];
      }
      delete [] _smoothedCells;

      _smoothedCells = createNewMap(_sizeX, _sizeZ);
      _sizeSmoothedX = _sizeX;
      _sizeSmoothedZ = _sizeZ;
   }

   loadRescaleInformation(true);
}

// resizes Map if it can not fit the coords
void HeightMap::rescaleMap(double min_x, double max_x, double min_z, double max_z)
{
   if(_sizeX == 0 || _sizeZ == 0) {    // first update
      int cellsX = (int)((max_x - min_x)/_resolution);
      int cellsZ = (int)((max_z - min_z)/_resolution);
      cellsX = 100 * ((cellsX / 100) + 1);      // up to next 100
      cellsZ = 100 * ((cellsZ / 100) + 1);
      double worldSizeX = cellsX * _resolution;
      double worldSizeZ = cellsZ * _resolution;
      double middleOffsetX = (worldSizeX - (max_x - min_x)) / 2.0;   // center map in world
      double middleOffsetZ = (worldSizeZ - (max_z - min_z)) / 2.0;   // center map in world
      _worldOffsetX = min_x - middleOffsetX;
      _worldOffsetZ = min_z - middleOffsetZ;
      _sizeX = cellsX;
      _sizeZ = cellsZ;
      _cells = createNewMap(_sizeX, _sizeZ);
      M_INFO1("Created HeightMap Size(%d x %d) from (%.2f, %.2f) to (%.2f, %.2f)\n", 
            _sizeX, _sizeZ, _worldOffsetX, _worldOffsetZ, (_sizeX - 1) * _resolution + _worldOffsetX, (_sizeZ - 1) * _resolution + _worldOffsetZ);
      return;
   }

   // Determine if rescale necessary
   double min_world_x, max_world_x, min_world_z, max_world_z;
   grid2worldMM(min_world_x, min_world_z, 0, 0);
   grid2worldMM(max_world_x, max_world_z, _sizeX - 1, _sizeZ - 1);

   bool rescale_x = false;
   bool rescale_z = false;
   if(min_x < min_world_x || max_x > max_world_x) {
      rescale_x = true;
      // dont rescale smaller than the old world
      if(min_world_x < min_x)
         min_x = min_world_x;
      if(max_world_x > max_x)
         max_x = max_world_x;
   }
   if(min_z < min_world_z || max_z > max_world_z) {
      rescale_z = true;
      if(min_world_z < min_z)
         min_z = min_world_z;
      if(max_world_z > max_z)
         max_z = max_world_z;
   }

   int newCellsX = _sizeX;
   int newCellsZ = _sizeZ;
   double newOffsetX = _worldOffsetX;
   double newOffsetZ = _worldOffsetZ;

   if(rescale_x) {
      newCellsX = (int)((max_x - min_x)/_resolution);
      newCellsX = 100 * ((newCellsX / 100) + 1);
      double worldSizeX = newCellsX * _resolution;
      double middleOffsetX = (worldSizeX - (max_x - min_x)) / 2.0;
      newOffsetX = min_x - middleOffsetX;
   }
   if(rescale_z) {
      newCellsZ = (int)((max_z - min_z)/_resolution);
      newCellsZ = 100 * ((newCellsZ / 100) + 1);
      double worldSizeZ = newCellsZ * _resolution;
      double middleOffsetZ = (worldSizeZ - (max_z - min_z)) / 2.0;
      newOffsetZ = min_z - middleOffsetZ;
   }

   if(rescale_x || rescale_z) {     // Actually Rescale
      saveRescaleInformation(true);

      int sX = _sizeX;
      int sZ = _sizeZ;
      if(rescale_x)
         sX = newCellsX;
      if(rescale_z)
         sZ = newCellsZ;
      HeightCell** newMap = createNewMap(sX, sZ);
      // Fill new Map
      for(int x = 0; x < _sizeX; x++) {
         for(int z = 0; z < _sizeZ; z++) {
            double world_x, world_z;
            grid2worldMM(world_x, world_z, x, z);
            int new_index_x = (int) ((world_x - newOffsetX) / _resolution);
            int new_index_z = (int) ((world_z - newOffsetZ) / _resolution);

            newMap[new_index_x][new_index_z] = _cells[x][z];
         }
      }

      // delete old map
      for(int i = 0; i < _sizeX; i++) {
         delete [] _cells[i];
      }
      delete [] _cells;

      _cells = newMap;
      _sizeX = sX;
      _sizeZ = sZ;
      _worldOffsetX = newOffsetX;
      _worldOffsetZ = newOffsetZ;

      M_INFO1("Rescaled HeightMap Size(%d x %d) from (%.2f, %.2f) to (%.2f, %.2f)\n", 
            _sizeX, _sizeZ, _worldOffsetX, _worldOffsetZ, (_sizeX - 1) * _resolution + _worldOffsetX, (_sizeZ - 1) * _resolution + _worldOffsetZ);

      double sizeMB = _sizeX * _sizeZ * sizeof(HeightCell) / 1024.0 / 1024.0;
      if (sizeMB > 2000.0) {
         M_ERR("Cannot allocate map requiring %1.2lf MB of memory! Please reduce the resolution.\n",sizeMB);
         exit(0);
      }
      else 
         printf("%1.2lf ",sizeMB);fflush(stdout);

      // there is/was a smoothed map, also resize
      if(_smoothedCells != NULL) {
         M_INFO1("Rescaling smoothed map to map size.\n");
         for(int i = 0; i < _sizeSmoothedX; i++) {
            delete [] _smoothedCells[i];
         }
         delete [] _smoothedCells;

         _smoothedCells = createNewMap(_sizeX, _sizeZ);
         _sizeSmoothedX = _sizeX;
         _sizeSmoothedZ = _sizeZ;
      }

      loadRescaleInformation(true);
   }
}
      
void HeightMap::loadRescaleInformation(bool verbose)
{
   HeightCell** cells = _cells;
   if(_smoothingEnabled) {
      cells = _smoothedCells;
   }
   if(verbose)
      M_INFO1("Loading rescale info size: %d\n", _rescaleInformation.size());
   for(unsigned int i = 0; i < _rescaleInformation.size(); i++) {
      rescaleInformation & ri = _rescaleInformation.at(i);
      int gx, gz;
      world2gridMM(ri.wx, ri.wz, gx, gz);
      if(pointInMap(gx, gz)) {
         cells[gx][gz].setClass(ri.cellClass, ri.isMrfClass);
      }
   }
}

/// Save information that might get lost during rescale.
/**
 * Currently save MRF classes.
 */
void HeightMap::saveRescaleInformation(bool verbose)
{
   _rescaleInformation.clear();

   int sx = _sizeX;
   int sz = _sizeZ;
   HeightCell** cells = _cells;
   if(_smoothingEnabled) {
      sx = _sizeSmoothedX;
      sz = _sizeSmoothedZ;
      cells = _smoothedCells;
   }
   for(int x = 0; x < sx; x++) {
      for(int z = 0; z < sz; z++) {
         if(cells[x][z].isMrfClassified()) {
            rescaleInformation ri;
            double wx, wz;
            grid2worldMM(wx, wz, x, z);
            ri.wx = wx;
            ri.wz = wz;
            ri.isMrfClass = cells[x][z].isMrfClassified();
            ri.cellClass = cells[x][z].getClass();
            _rescaleInformation.push_back(ri);
         }
      }
   }
   if(verbose)
      M_INFO1("Save rescale Information size: %d.\n", _rescaleInformation.size());
}

void HeightMap::createSmoothedMap()
{
   if(_smoothedCells == NULL) {
      _smoothedCells = createNewMap(_sizeX, _sizeZ);
      _sizeSmoothedX = _sizeX;
      _sizeSmoothedZ = _sizeZ;
      M_INFO1("Creating Smoothed map Size (%d x %d).\n", _sizeX, _sizeZ);
   } else if((_sizeSmoothedX != _sizeX) || (_sizeSmoothedZ != _sizeZ)) {
      M_WARN("Smoothed Map has another size than the map - this should never happen, not updating.\n");
      return;
   }

   //M_INFO1("Smoothing Map.\n");
   Timing t1("Smoothing");
   for(int x = 0; x < _sizeX; x++) {
      for(int z = 0; z < _sizeZ; z++) {
         if(SMOOTH_DO_MEDIAN)
            medianCell(x, z);
         else
            smoothCell(x, z);
      }
   }
   t1.printInfo();

#ifdef DO_HEIGHTMAP_TIMING
   static double sumDiff = 0;
   static double sumDiffSqr = 0;
   static double numDiff = 0;
   static bool in = false;
   double diff = t1.diff();
   printf("diff: %.2f\n", diff*1000);
   if(in && diff > 5.0/1000.0) {  // dont coutn empty map
      numDiff += 1.0;
      sumDiff += diff;
      sumDiffSqr += diff*diff;
      double mean = sumDiff/numDiff;
      double std = sqrt(1/numDiff * (sumDiffSqr - sumDiff*sumDiff/numDiff));
      printf("Smooth: %.2fms +- %.2fms\n", mean*1000, std*1000);
   }
   in = true;
#endif
}

void HeightMap::swapWH(struct weightedHeight* w1, struct weightedHeight* w2)
{
   double weight1 = w1->weight;
   double height1 = w1->height;
   w1->weight = w2->weight;
   w1->height = w2->height;
   w2->weight = weight1;
   w2->height = height1;
}

/// Performs stupid bubblesort -> hopefully faster for this small numbers
void HeightMap::sortWeightedHeights(int num, struct weightedHeight * whs)
{
   if(num <= 1)
      return;
   // num >= 2
   for(int j = 0; j < num - 1; j++) {
      bool neededSwap = false;
      // let a bubble go up
      for(int i = 1; i < num - j; i++) {
         if(whs[i] < whs[i - 1]) {
            swapWH(&whs[i], &whs[i - 1]);
            neededSwap = true;
         }
      }
      if(!neededSwap)
         return;
   }
}

///\todo 0 zellen verhindern
void HeightMap::medianCell(int x, int z)
{
   // static -> no reallocations
   static struct weightedHeight weightedHeight[9];

   double newHeight = _cells[x][z].getHeight();
   double newVariance = _cells[x][z].getVariance();
   if(newHeight == 0.0)
      newVariance = 1000*1000;
   bool uninitializedCell = false;
   if((newHeight == 0) && (newVariance >= 500*500))
      uninitializedCell = true;
   int initializedNeighbors = 0;   // number of neighbor cells, that are initialized.

   double sumWeight = 0;
   int count = 0;
   for(int xx = x - 1; xx <= x + 1; xx++) {
      if(xx < 0)
         continue;
      if(xx >= _sizeX)
         continue;
      for(int zz = z - 1; zz <= z + 1; zz++) {
         if(zz < 0)
            continue;
         if(zz >= _sizeZ)
            continue;

         weightedHeight[count].weight = 1.0/_cells[xx][zz].getVariance();
         if((xx == x) && (zz == z))
            weightedHeight[count].weight *= 2.0;
         weightedHeight[count].height = _cells[xx][zz].getHeight();
         if(weightedHeight[count].height == 0)
            weightedHeight[count].weight = 0;
         if(weightedHeight[count].height != 0)
            initializedNeighbors++;
         else if(_cells[xx][zz].getVariance() < 500.0 * 500.0)
            initializedNeighbors++;
         sumWeight += weightedHeight[count].weight;
         count++;
      }
   }

   sortWeightedHeights(count, weightedHeight);
   
   double sumReached = 0;
   sumWeight /= 2.0;
   for(int i = 0; i < count; i++) {
      sumReached += weightedHeight[i].weight;
      if(sumReached >= sumWeight) {
         if(!uninitializedCell || initializedNeighbors >= 1)   // was 5
            newHeight = weightedHeight[i].height;
         break;
      }
   }

   _smoothedCells[x][z].integrateMeasurement(newHeight, newVariance, _cells[x][z].getLastUpdateTime(), true);
}

void HeightMap::smoothCell(int x, int z)
{
   double weight_sum = 0;
   double sum_weight = 0;
   double cellHeight = _cells[x][z].getHeight();

   int allAroundHigher = 0;
   for(int xx = x - 1; xx <= x + 1; xx++) {
      if(xx < 0)
         continue;
      if(xx >= _sizeX)
         continue;
      for(int zz = z - 1; zz <= z + 1; zz++) {
         if(zz < 0)
            continue;
         if(zz >= _sizeZ)
            continue;
        
         double height = _cells[xx][zz].getHeight();
         if(height > cellHeight)
            allAroundHigher++;
         double mahalab = _cells[x][z].sqrMahalanobisDistance(height);
         if((fabs(height - cellHeight) < 50.0) && (mahalab > (MAX_SMOOTH_MAHALNOBIS_DIST * MAX_SMOOTH_MAHALNOBIS_DIST)))   // dont smooth if cells too far apart and mahaln. too big
            continue;
         // ie smooth if cell difference small enough, or mahalan. small enough

         //double w = _cells[xx][zz].getInvSigmaFast();
         double w = 1.0/_cells[xx][zz].getVariance();
         if(w > 100)
            w = 100;

         double w2 = abs(xx - x) + abs(zz - z);
         if(w2 == 0) {
            w2 = 1;
         } else if(w2 == 1) {
            w2 = 0.5;
         } else if(w2 == 2) {
            w2 = 0.25;
         } else {
            M_WARN("%s: weight unusual: %.2f \n", __func__, w2);
            w2 = 1/(w2 * w2);
         }
         sum_weight += w * w2;
         weight_sum += w * w2 * height;
      }
   }

   // cells around this cell higher -> fill hole by weighted median
   if(allAroundHigher >= 5) {
      vector<double> heights;
      for(int xx = x - 1; xx <= x + 1; xx++) {
         if(xx < 0)
            continue;
         if(xx >= _sizeX)
            continue;
         for(int zz = z - 1; zz <= z + 1; zz++) {
            if(zz < 0)
               continue;
            if(zz >= _sizeZ)
               continue;
            
            heights.push_back(_cells[xx][zz].getHeight());
            if(zz == z && xx == x)  // middle twice
               heights.push_back(_cells[xx][zz].getHeight());
         }
      }
      std::sort(heights.begin(), heights.end());
      sum_weight = 1;
      weight_sum = heights.at(heights.size() / 2);
   }

   double newHeight = 0;
   double newVariance = 1000*1000;
   if(sum_weight > 0) {
      newHeight = weight_sum/sum_weight;
      if(DO_NOT_SMOOTH_VARIANCES)
         newVariance = _cells[x][z].getVariance();
      else
         newVariance = 1/sum_weight;
   } else {
      newHeight = _cells[x][z].getHeight();
      newVariance = _cells[x][z].getVariance();
   }

   _smoothedCells[x][z].integrateMeasurement(newHeight, newVariance, _cells[x][z].getLastUpdateTime(), true); 
}



void HeightMap::world2gridMM(double worldx, double worldz, int & gridx, int & gridz) const
{
   gridx = (int) floor((worldx - _worldOffsetX) / _resolution);   ///\todo: FIXME floor has to be set
   gridz = (int) floor((worldz - _worldOffsetZ) / _resolution); 
}

void HeightMap::grid2worldMM(double & worldx, double & worldz, int gridx, int gridz) const
{
   worldx = gridx * _resolution + _worldOffsetX;
   worldz = gridz * _resolution + _worldOffsetZ;
}

int HeightMap::indexFromGridCoords(int x, int z) const
{
   if((_sizeX != _sizeSmoothedX) || (_sizeZ != _sizeSmoothedZ)) {
      if((_sizeSmoothedZ != 0) || (_sizeSmoothedX != 0))
         M_WARN("%s: sizes between smoothed and non-smoothed map not equal. (%dx%d - %dx%d)\n", __func__, _sizeX, _sizeZ, _sizeSmoothedX, _sizeSmoothedZ);
   }
   return (x + z * _sizeX);
}

void HeightMap::gridCoordsFromIndex(int index, int & x, int & z) const
{
   if((_sizeX != _sizeSmoothedX) || (_sizeZ != _sizeSmoothedZ)) {
      if((_sizeSmoothedZ != 0) || (_sizeSmoothedX != 0))
         M_WARN("%s: sizes between smoothed and non-smoothed map not equal. (%dx%d - %dx%d)\n", __func__, _sizeX, _sizeZ, _sizeSmoothedX, _sizeSmoothedZ);
   }
   z = index / _sizeX;
   x = index % _sizeX;
}

bool HeightMap::validIndex(int index) const
{
   if(index < 0)
      return false;
   if(index >= _sizeX * _sizeZ)
      return false;

   return true;
}

bool HeightMap::pointInMap(int x, int z) const
{
   if((x < 0) || (z < 0))
      return false;
   if((x >= sizeX()) || (z >= sizeZ()))
      return false;
   return true;
}

void HeightMap::classifyMap()
{
   Timing t("Classify");
   _classifier->simpleClassifyMap(this);
   t.printInfo();

#ifdef DO_HEIGHTMAP_TIMING
   static double sumDiff = 0;
   static double sumDiffSqr = 0;
   static double numDiff = 0;
   static bool in = false;
   double diff = t.diff();
   printf("diff: %.2f\n", diff*1000);
   if(in && diff > 5.0/1000.0) {  // dont coutn empty map
      numDiff += 1.0;
      sumDiff += diff;
      sumDiffSqr += diff*diff;
      double mean = sumDiff/numDiff;
      double std = sqrt(1/numDiff * (sumDiffSqr - sumDiff*sumDiff/numDiff));
      printf("Classify: %.2fms +- %.2fms\n", mean*1000, std*1000);
   }
   in = true;
#endif
}

short HeightMap::clampToShort(double d) const
{
   if(d > SHRT_MAX)
      return SHRT_MAX;
   if(d < SHRT_MIN)
      return SHRT_MIN;

   return (short)d;
}


bool HeightMap::save(string filename)
{
    string fname = filename;
    std::ofstream os(fname.c_str());

    if (!os.good()) {
       M_ERR("Cannot open file for writing %s\n",fname.c_str());
       return false;
    }
    if (_cells == NULL)
       return false;

    os << HEIGHTMAP_VERSION << endl;

    os << _rescaleInformation.size() << endl;
    for (unsigned i=0; i< _rescaleInformation.size(); i++) 
    {
       os << _rescaleInformation[i].wx << " ";
       os << _rescaleInformation[i].wz << " "; 
       os << _rescaleInformation[i].isMrfClass << " ";
       os << _rescaleInformation[i].cellClass << " ";
    }

    os << endl;

    os << _id << " " << _sizeX << " " << _sizeZ << " " << _worldOffsetX << " " << _worldOffsetZ << " ";
    os << _resolution << " " << _sizeSmoothedX << " " <<_sizeSmoothedZ << " " << _smoothingEnabled << " ";

    os << endl;

    os << endl;
    M_INFO1("Writing %d cells to file\n", _sizeX * _sizeZ);
    for (int x=0; x<_sizeX; x++) {
       for (int z=0; z<_sizeZ; z++) {
          _cells[x][z].writeToStream(os);
       }
    }

    M_INFO1("Saved hight map to %s\n",fname.c_str());
    return true;
}


bool HeightMap::loadFromTIFF(unsigned int* buffer, int width, int height, double resolution, double heightResolution, bool inverse, unsigned int* mask)
{
   if (buffer == NULL) {
      M_ERR("Got empty TIFF buffer!\n");
      return false;
   }

   // Delete old map
   if (_cells != NULL) {
      for(int i = 0; i < _sizeX; i++) {
         delete [] _cells[i];
      }
      delete [] _cells;
   }

   _sizeX = width;
   _sizeZ = height;
   _worldOffsetX = 0.0; 
   _worldOffsetZ = 0.0;

   // Convert to mm
   _resolution = resolution * 1000.0; 

   _heightResolution = heightResolution * 1000.0;

   gettimeofday(&_lastUpdateTime, 0);

   unsigned char* b = (unsigned char*) buffer;
   //int maxval = 255;

   _cells = createNewMap(_sizeX, _sizeZ);
   double variance = (50.0 * 50.0);
   double lowest = HUGE_VAL;
   double highest = -HUGE_VAL;
   for (int x=0; x<_sizeX; x++) {
      for (int z=0; z<_sizeZ; z++) {
         double pointHeight = 0.0;
         int idx = 4*(z*_sizeX + x);
         int red = 4*(z*_sizeX + x) ;
         int green = 4*(z*_sizeX + x) + 1;
         int blue = 4*(z*_sizeX + x) + 2;
         //if (b[idx] <= maxval) 
         {
            //printf ("%d ",b[idx]);
            if (inverse)
               pointHeight = heightResolution * 1000.0 * (double) (255 - b[idx]); 
            else
               pointHeight = heightResolution * 1000.0 * (double) b[idx]; 
            //pointHeight = 1000.0 * 10 * ( (double) b[idx] / (double) maxval ) ;
            //printf ("%lf ",pointHeight);
            if (pointHeight > highest)
               highest = pointHeight;
            if (pointHeight < lowest)
               lowest = pointHeight;

            _cells[x][z].integrateMeasurement(pointHeight, variance, _lastUpdateTime, true);
            if (mask != NULL) {
               unsigned char* b2 = (unsigned char*) mask;

               if (b2[red] > 100 && b2[green] > 100 && b2[blue] > 100) 
                  _cells[x][z].setIsMasked(false);
               else
                  _cells[x][z].setIsMasked(true);

               /*if ((b2[red] < 200 && b2[green] < 200 && b2[blue] < 200) || 
                  (b2[red] > 100 && b2[green] < 200 && b2[blue] < 200))
                  _cells[x][z].setIsMasked(true);
               else
                  _cells[x][z].setIsMasked(false);
            */
               
               // TODO set here real vegetation density degree
               if ((b2[red] < 120 && b2[green] > 200 && b2[blue] < 120) )
                  _cells[x][z].setVegetation(100);
            }
         }
      }
   }
   printf("Highest peak: %1.2lf m   Lowest Valley: %1.2lf m\n",highest/1000.0, lowest/1000.0);
   return true;
}


bool HeightMap::loadFromHeightMap(HeightMap* hm)
{
   if (hm== this) 
      return false;

   // Delete old map
   if (_cells != NULL) {
      for(int i = 0; i < _sizeX; i++) {
         delete [] _cells[i];
      }
      delete [] _cells;
   }

   _sizeX = hm->sizeX();;
   _sizeZ = hm->sizeZ();;
   _worldOffsetX = 0.0; 
   _worldOffsetZ = 0.0;
   _resolution = hm->resolutionMM(); 

   gettimeofday(&_lastUpdateTime, 0);

   _cells = createNewMap(_sizeX, _sizeZ);
   double variance = (50.0 * 50.0);
   for (int x=0; x<_sizeX; x++) {
      for (int z=0; z<_sizeZ; z++) {
            double pointHeight = hm->getCellsMM()[x][z].getHeight(); 
            _cells[x][z].integrateMeasurement(pointHeight, variance, _lastUpdateTime, true);
      }
   }
   return true;
}



bool HeightMap::load(string filename)
{
   string fname = filename;
   std::ifstream is(fname.c_str());

   if (!is.good()) {
      M_ERR("Cannot open file for reading %s\n",fname.c_str());
      return false;
   }

   // Delete old map
   if (_cells != NULL) {
      for(int i = 0; i < _sizeX; i++) {
         delete [] _cells[i];
      }
      delete [] _cells;
   }

   string ident;
   is >> ident;

   if (ident != HEIGHTMAP_VERSION) {
      M_ERR("Not a hight map file or wrong version!\n");
      return false;
   } 

   unsigned int s;
   is >> s;
   _rescaleInformation.clear();
   for (unsigned i=0; i<s; i++) 
   {
      int dummy;
      rescaleInformation ri; 
      is >> ri.wx;
      is >> ri.wz; 
      is >> ri.isMrfClass;
      is >> dummy;
      ri.cellClass = (HeightCell::ELEVATION_CLASSES) dummy;
      _rescaleInformation.push_back(ri);
   }


   is >> _id;
   is >> _sizeX; 
   is >> _sizeZ; 
   is >> _worldOffsetX; 
   is >> _worldOffsetZ;
   is >> _resolution; 
   is >> _sizeSmoothedX;
   is >> _sizeSmoothedZ; 
   is >> _smoothingEnabled;

   gettimeofday(&_lastUpdateTime, 0);

   _cells = createNewMap(_sizeX, _sizeZ);
   for (int x=0; x<_sizeX; x++) {
      for (int z=0; z<_sizeZ; z++) {
         _cells[x][z].readFromStream(is);
      }
   }

   // TODO: HeightMapClassifier* _classifier;
   M_INFO1("Loaded hight map from %s\n",fname.c_str());

   return true;
}


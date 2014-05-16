#ifndef HEIGHTMAP_H
#define HEIGHTMAP_H

#include "heightmap/heightcell.h"
#include "heightmap/heightMapClassifier.h"
#include "utilities/math/vector4.h"
#include "heightmap/scan3d.h"
#include <vector>

using std::vector;

// A Map of heightcells
class HeightMap
{
   struct weightedHeight
   {
      double weight;
      double height;
      bool operator<(const weightedHeight & wh)
      {
         return (height < wh.height);
      }
   };

   // IMPORTANT: Interface uses meters, whereas internally the class uses mm !!!
   //            Please adjust any non-privat function accordingly!
   public:
      HeightMap(double resolution = 0.05, double worldOffsX = 0, double worldOffsZ = 0, int sizeX = 0, int sizeZ = 0, bool mrfWeightsLoadDebug = true);
      ~HeightMap();

      bool save(string filename);
      bool load(string filename);
      bool loadFromTIFF(unsigned int* buffer, int width, int height, double resolution, double heightResolution, bool inverse=false, unsigned int* mask=NULL);
      bool loadFromHeightMap(HeightMap* hm);
      void classifyMap();
      HeightMapClassifier* getClassifier();

      // Expects the 3d-coordinates to be world coordinates values in meters
      void updateFromWorldScan(const Scan3D & scan);
      void updateFromPointMM(const Vector3d& p);

      void createSmoothedMap();
      int smoothedSizeX() const { return _sizeSmoothedX; };
      int smoothedSizeZ() const { return _sizeSmoothedZ; };
      bool smoothingEnabled() { return _smoothingEnabled; };
      void setSmoothing(bool enable);
  
      int id() const { return _id; };
      struct timeval lastUpdateTime() const { return _lastUpdateTime; };

      // Values in meters
      double worldOffsetX() const { return _worldOffsetX / 1000.0; };
      double worldOffsetZ() const { return _worldOffsetZ / 1000.0; };
      double resolution() const { return _resolution / 1000.0; };
      double hresolution() const { return _heightResolution / 1000.0; };
      void world2grid(double worldx, double worldz, int & gridx, int & gridz) const 
                      {world2gridMM(1000.0*worldx, -1000.0* worldz, gridx, gridz);}
      void grid2world(double & worldx, double & worldz, int gridx, int gridz) const
                      {grid2worldMM(worldx, worldz, gridx, gridz); worldx/=1000.0; worldz/=-1000.0;}


      // Values in grid cells
      int sizeX() const { return _sizeX; };
      int sizeZ() const { return _sizeZ; };
      bool pointInMap(int x, int z) const;
      int indexFromGridCoords(int x, int z) const;
      bool validIndex(int index) const;
      void gridCoordsFromIndex(int index, int & x, int & z) const;

      bool is_cell_traversable(int x,int y) {
          if ( this->getCellsMM()[x][y].getClass() 
                == HeightCell::ELC_FLAT_GROUND 
            || this->getCellsMM()[x][y].getClass() 
                == HeightCell::ELC_RAMPED_GROUND
            || this->getCellsMM()[x][y].getClass() 
                == HeightCell::ELC_DRIVABLE_OBSTACLE
            || this->getCellsMM()[x][y].getClass() 
                == HeightCell::ELC_STAIRS ) {
                return true;
           } else {
               return false;
           }
      }

      // Values in millimeters
      HeightCell** getCellsMM();
      HeightCell** getSmoothedCellsMM();
      void world2gridMM(double worldx, double worldz, int & gridx, int & gridz) const;
      void grid2worldMM(double & worldx, double & worldz, int gridx, int gridz) const;
      double worldOffsetXMM() const { return _worldOffsetX; };
      double worldOffsetZMM() const { return _worldOffsetZ; };
      double resolutionMM() const { return _resolution; };
 

   private:

      /// Deletes the _smoothedCells array
      void deleteSmoothedMap();

      double meanErrorToWorldScan(const Scan3D & scan);
      void rescaleMap(double min_x, double max_x, double min_z, double max_z);
      /// Rescale map, but keep the total area clamped.
      void clampedRescaleMap(double min_x, double max_x, double min_z, double max_z);

      void setId(int id) { _id = id; };
      void setLastUpdateTime(struct timeval time) { _lastUpdateTime = time; };   ///< Do not set this manually, unless you know what you're doing.

   protected:
      void saveRescaleInformation(bool verbose);
      void loadRescaleInformation(bool verbose);
      void swapWH(struct weightedHeight* w1, struct weightedHeight* w2);
      void sortWeightedHeights(int num, struct weightedHeight * whs);
      HeightCell** createNewMap(int sx, int sz) const;
      short clampToShort(double d) const;
      void smoothCell(int x, int z);
      void medianCell(int x, int z);

   protected:
      struct rescaleInformation {
         double wx, wz; ///< World coordinates.
         bool isMrfClass;
         enum HeightCell::ELEVATION_CLASSES cellClass;
      };
      vector<struct rescaleInformation> _rescaleInformation; 
      int _id;
      int _sizeX;
      int _sizeZ;
      double _worldOffsetX;
      double _worldOffsetZ;
      double _resolution;     // size of a grid cell in mm
      double _heightResolution;     // size of a grid cell in mm
      HeightCell** _cells;
      int _sizeSmoothedX;
      int _sizeSmoothedZ;
      HeightCell** _smoothedCells;

      HeightMapClassifier* _classifier;

      struct timeval _lastUpdateTime;
      bool _smoothingEnabled;
   
      double _lastRescaleRobotX;
      double _lastRescaleRobotZ;

      bool _classifyCompleteUpdateAsWall;
};

#endif


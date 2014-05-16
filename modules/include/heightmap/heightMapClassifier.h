#ifndef HEIGHTMAP_CLASSIFIER_H
#define HEIGHTMAP_CLASSIFIER_H

#include "utilities/math/vector3d.h"
#include "heightmap/heightcell.h"
#include "utilities/paramfile.h"

class HeightMap;
class ElevationMrfGraph;


/// Classifies a heightmap's cell array.
class HeightMapClassifier
{
   public:
      HeightMapClassifier(bool printLoadInfo);
      ~HeightMapClassifier();

      void simpleClassifyMap(HeightMap * const map);

      int curSizeX() const;
      int curSizeZ() const;
      HeightCell** curCells() const;
      HeightCell** curModifieableCells();


      /// Computes normals for the heightcells, and eventually preclassifies map.
      void prepareMap(HeightMap * const map, bool doPreclassify);
      /// Create a MRF Graph from the current map.
      // void curMapToMRF();
      //ElevationMrfGraph* getElevationMrfGraph();

      /// Compute up to 4 normal vectors around x,z and the mid points of the triangles as a normal start point.
      Vector3d pointNormalsAround(int x, int z, std::vector<Vector3d> & midPoints, std::vector<Vector3d> & normals) const;

      enum HeightCell::ELEVATION_CLASSES simpleClassifyCell(int x, int z) const;
      enum HeightCell::ELEVATION_CLASSES simpleClassifyHeightValue(double height) const;
      /// Definate classification, override is true, if it is safe to override MRF.
      enum HeightCell::ELEVATION_CLASSES safeClassifyCell(int x, int z, bool & override);

      void computeNormals();
      void addFeatures();

      bool cellUninitialized(int x, int z) const;
      double maxDeltaAround(int x, int z) const;
      double avgDeltaAround(int x, int z) const;

      Vector3d getNormal(int x1, int z1, int x2, int z2, int x3, int z3) const;
      Vector3d getMidPoint(int x1, int z1, int x2, int z2, int x3, int z3) const;
      void getAvgPitchAround(int x, int z, double & pitch, double & maxPitchDelta) const;

   protected:
      HeightMap* _curMap;
      ElevationMrfGraph* _elevationMrfGraph;
      double _minWallHeight;
      double _flatHeight;
      double _minRampAngle;
      double _maxRampAngle;

};

#endif


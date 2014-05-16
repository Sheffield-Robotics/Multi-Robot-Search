#include "heightmap/heightMapClassifier.h"
#include "utilities/misc.h"
#include "utilities/timing.h"
#include "heightmap/heightmap.h"
#include <string.h>
#include "utilities/paramfile.h"

#define HIGH_VARIANCE (500 * 500)

HeightMapClassifier::HeightMapClassifier(bool printLoadInfo)
{
   (void) printLoadInfo;
   _curMap = NULL;
   //_elevationMrfGraph = new ElevationMrfGraph();
   char* rescuePath = getenv("RESCUE");
   if(rescuePath != NULL) {
      char buf[4096];
      strcpy(buf, rescuePath);
      strcat(buf, "/mrfHeightmapClassifier/config/weights.xml");
      /*if(!_elevationMrfGraph->loadWeightsFromFile(buf)) {
         rlogWarning("Could not load MRF weights from %s\n", buf);
      } else {
         if(printLoadInfo)
            rlogInfo("MRF weights loaded from %s\n", buf);
      }*/
   }

   // Initialize parameters
   _minWallHeight = Params::g_minWallHeight * 1000.0;// * 0.1;
   _flatHeight = Params::g_flatHeight * 1000.0;// * 0.1;
   _minRampAngle = Params::g_minRampAngle;
   _maxRampAngle = Params::g_maxRampAngle;
}

HeightMapClassifier::~HeightMapClassifier()
{
}

int HeightMapClassifier::curSizeX() const
{
   if(_curMap->smoothingEnabled())
      return _curMap->smoothedSizeX();
   else
      return _curMap->sizeX();
}

int HeightMapClassifier::curSizeZ() const
{
   if(_curMap->smoothingEnabled())
      return _curMap->smoothedSizeZ();
   else
      return _curMap->sizeZ();
}
   
HeightCell** HeightMapClassifier::curCells() const
{
   if(_curMap->smoothingEnabled())
      return _curMap->getSmoothedCellsMM();
   else
      return _curMap->getCellsMM();
}

HeightCell** HeightMapClassifier::curModifieableCells()
{
   if(_curMap->smoothingEnabled()) {
      return _curMap->getSmoothedCellsMM();
   } else {
      return _curMap->getCellsMM();
   }
}

/*ElevationMrfGraph* HeightMapClassifier::getElevationMrfGraph()
{
   return _elevationMrfGraph;
}*/

/**
 * Preclassify just detects no initialized cells and classifies them accordingly.
 * It does not change the class of other cells.
 */
void HeightMapClassifier::prepareMap(HeightMap * const map, bool doPreclassify)
{
   _curMap = map;

   Timing t("prepareMap");

   computeNormals();
   addFeatures();

   if(doPreclassify) {
      for(int x = 0; x < curSizeX(); x++) {
         for(int z = 0; z < curSizeZ(); z++) {
            if(cellUninitialized(x, z)) {
               //curModifieableCells()[x][z].setClass(HeightCell::ELC_NOT_INITIALIZED);
               curModifieableCells()[x][z].setClass(HeightCell::ELC_FLAT_GROUND);
            }
         }
      }
   }

   t.printInfo();
}

/*void HeightMapClassifier::curMapToMRF()
{
   _elevationMrfGraph->generateMapGraph(_curMap);
}*/

/**
 * Computes the cell's point normal vector and stores it in the metadata at position 0,1,2 für x,y,z. Also clears the cells metadata.
 */
void HeightMapClassifier::computeNormals()
{
   vector<Vector3d> mp;
   vector<Vector3d> pn;
   for(int x = 0; x < curSizeX(); x++) {
      for(int z = 0; z < curSizeZ(); z++) {
         mp.clear();
         pn.clear();
         Vector3d pointNormal = pointNormalsAround(x, z, mp, pn);
         curModifieableCells()[x][z].getMetaData().clear();
         curModifieableCells()[x][z].getMetaData().push_back(pointNormal.x);
         curModifieableCells()[x][z].getMetaData().push_back(pointNormal.y);
         curModifieableCells()[x][z].getMetaData().push_back(pointNormal.z);
      }
   }
}

/**
 * Currently pushes maxDeltaAround to metadata.
 */
void HeightMapClassifier::addFeatures()
{
   for(int x = 0; x < curSizeX(); x++) {
      for(int z = 0; z < curSizeZ(); z++) {
         double maxDelta = maxDeltaAround(x, z);
         curModifieableCells()[x][z].getMetaData().push_back(maxDelta);
      }
   }
}

// Note cells might be classified from previous step -> reset vorher falls nachbarklasse becahtet wird
// preclassify noetig:
// 1) features (normalen in zellen) brechenen, NICHT feautres nennen sondern metadata, anderes als mrf features!!!
// 2) preclassify noetig, damit nur initialisierte zellen hineinkommen
void HeightMapClassifier::simpleClassifyMap(HeightMap * const map)
{
   _curMap = map;
   // AK: changed the units of min wall height and flat height to meter
   // previously was treated like pixels; below converted to millimeter.
   _minWallHeight = Params::g_minWallHeight * 1000.0;// * _curMap->hresolution();
   _flatHeight = Params::g_flatHeight * 1000.0;// * _curMap->hresolution();
   _minRampAngle = Params::g_minRampAngle;
   _maxRampAngle = Params::g_maxRampAngle;


   computeNormals();

   for(int x = 0; x < curSizeX(); x++) {
      for(int z = 0; z < curSizeZ(); z++) {
         HeightCell::ELEVATION_CLASSES cl = HeightCell::ELC_FLAT_GROUND;
         if (Params::g_classify_everything_as_flat_ground) {
            cl = HeightCell::ELC_FLAT_GROUND;
         }
         else if (Params::g_classify_from_mask_only) {
            if(curCells()[x][z].getIsMasked()) 
               cl = HeightCell::ELC_WALL;
            else
               cl = HeightCell::ELC_FLAT_GROUND;
         }
         else {
            cl = simpleClassifyCell(x, z);
         }
         curModifieableCells()[x][z].setClass(cl);

         if(curModifieableCells()[x][z].isMrfClassified()) {
            M_INFO2("CELL IS MRF CLASSIFIED!!!!!\n");
            bool override = false;
            enum HeightCell::ELEVATION_CLASSES cls = safeClassifyCell(x, z, override);
            if(override)
               curModifieableCells()[x][z].setClass(cls, true);
         }
      }
   }
}

enum HeightCell::ELEVATION_CLASSES HeightMapClassifier::simpleClassifyCell(int x, int z) const
{
   if(cellUninitialized(x, z))
      return HeightCell::ELC_NOT_INITIALIZED;

   double maxDelta = maxDeltaAround(x, z);
   double avgDelta = avgDeltaAround(x, z);

   if(maxDelta > _minWallHeight) {
      return HeightCell::ELC_WALL;
   }

   if(avgDelta > _flatHeight || maxDelta > 1.5 * _flatHeight) {
      return HeightCell::ELC_DRIVABLE_OBSTACLE;
   } 

   if(maxDelta <= _flatHeight) {
      return HeightCell::ELC_FLAT_GROUND;
   }
   return HeightCell::ELC_DRIVABLE_OBSTACLE;

   double avgPitch, maxPitchDelta;
   getAvgPitchAround(x, z, avgPitch, maxPitchDelta);
   if((avgPitch >= _minRampAngle) && (avgPitch <= _maxRampAngle) && maxPitchDelta <= 15.0) {
      return HeightCell::ELC_DRIVABLE_OBSTACLE;
      //return HeightCell::ELC_RAMPED_GROUND;
   }
   
   if(avgPitch > _maxRampAngle && avgDelta > 5.0)    // too steep, don't drive here
      return HeightCell::ELC_DRIVABLE_OBSTACLE;

   if(avgDelta <= _flatHeight) {
      return HeightCell::ELC_FLAT_GROUND;
   }

   return HeightCell::ELC_UNCLASSIFIED;
}

enum HeightCell::ELEVATION_CLASSES HeightMapClassifier::simpleClassifyHeightValue(double height) const
{
   double maxDelta = height;
   double avgDelta = height;

   if(maxDelta > _minWallHeight) {
      return HeightCell::ELC_WALL;
   }

   if(avgDelta > _flatHeight || maxDelta > 1.5 * _flatHeight) {
      return HeightCell::ELC_DRIVABLE_OBSTACLE;
   } 

   if(maxDelta <= _flatHeight) {
      return HeightCell::ELC_FLAT_GROUND;
   }
   return HeightCell::ELC_DRIVABLE_OBSTACLE;
}


/**
 * Classify safely to override MRF class because map might have changed.
 * Just applied to OBSTACLE cells that are very flat now.
 */
enum HeightCell::ELEVATION_CLASSES HeightMapClassifier::safeClassifyCell(int x, int z, bool & override)
{
   override = false;
   double md = maxDeltaAround(x, z);
   if(md > 250.0) {  // definately a wall.
      override = true;
      M_INFO1("Overriding MRF to wall at (%d, %d)\n", x, z);
      return HeightCell::ELC_WALL;
   }

   if(curCells()[x][z].getClass() != HeightCell::ELC_DRIVABLE_OBSTACLE) {
      return curCells()[x][z].getClass();
   }
   if(md <= 10.0) {  // override MRF only if the cell is really flat.
      override = true;
      M_INFO1("Overriding MRF from obstacle to flat at (%d, %d)\n", x, z);
      return HeightCell::ELC_FLAT_GROUND;
   }
   return curCells()[x][z].getClass();
}

bool HeightMapClassifier::cellUninitialized(int x, int z) const
{
   if(curCells()[x][z].getVariance() > HIGH_VARIANCE) {
      if(curCells()[x][z].getHeight() == 0.0) {               // Hohe Varianz und Hoehe exakt 0.0 -> noch kein update
         return true;
      }
   }
   return false; 
}

double HeightMapClassifier::maxDeltaAround(int x, int z) const
{
   int rad = 2;
   double maxDelta = 0;
   for(int xx = x - rad; xx <= x + rad; xx++) {
      if((xx < 0) || (xx >= curSizeX()))
         continue;
      for(int zz = z - rad; zz <= z + rad; zz++) {
         if((zz < 0) || (zz >= curSizeZ()))
            continue;

         if(cellUninitialized(xx, zz))
            continue;

         double delta = fabs(curCells()[x][z].getSampleHeight() - curCells()[xx][zz].getSampleHeight());
         if(delta > maxDelta)
            maxDelta = delta;
      }
   }
   return maxDelta;
}
   
double HeightMapClassifier::avgDeltaAround(int x, int z) const
{
   double avgDelta = 0;
   double numDeltas = 0;
   for(int xx = x - 1; xx <= x + 1; xx++) {
      if((xx < 0) || (xx >= curSizeX()))
         continue;
      for(int zz = z - 1; zz <= z + 1; zz++) {
         if((zz < 0) || (zz >= curSizeZ()))
            continue;

         if(cellUninitialized(xx, zz))
            continue;

         double delta = fabs(curCells()[x][z].getSampleHeight() - curCells()[xx][zz].getSampleHeight());
         avgDelta += delta;
         numDeltas += 1.0;
      }
   }
   if(numDeltas > 0)
      avgDelta /= numDeltas;
   return avgDelta;
}

/**
 * \param [in] x,z The indices of the point.
 * \param [out] midPoints The midpoints of the triangles corresponding to the normals.
 * \param [out] normals The normal vectors.
 * \return The point normal vector.
 */
Vector3d HeightMapClassifier::pointNormalsAround(int x, int z, vector<Vector3d> & midPoints, vector<Vector3d> & normals) const
{
   // links oben
   if((x - 1 >= 0) && (z - 1 >= 0)) {
      if(!cellUninitialized(x - 1, z) && !cellUninitialized(x, z - 1)) {
         Vector3d nLo = getNormal(x, z, x, z - 1, x - 1, z);
         midPoints.push_back(getMidPoint(x, z, x - 1, z, x, z - 1));
         normals.push_back(nLo);
      }
   }

   // links unten
   if((x - 1 >= 0) && (z + 1 < curSizeZ())) {
      if(!cellUninitialized(x, z + 1) && !cellUninitialized(x - 1, z)) {
         Vector3d nLu = getNormal(x, z, x - 1, z, x, z + 1);
         midPoints.push_back(getMidPoint(x, z, x, z + 1, x - 1, z));
         normals.push_back(nLu);
      }
   }

   // rechts unten
   if((x + 1 < curSizeX()) && (z + 1 < curSizeZ())) {
      if(!cellUninitialized(x + 1, z) && !cellUninitialized(x, z + 1)) {
         Vector3d nRu = getNormal(x, z, x, z + 1, x + 1, z);
         midPoints.push_back(getMidPoint(x, z, x + 1, z, x, z + 1));
         normals.push_back(nRu);
      }
   }

   // rechts oben
   if((x + 1 < curSizeX()) && (z - 1 >= 0)) {
      if(!cellUninitialized(x, z - 1) && !cellUninitialized(x + 1, z)) {
         Vector3d nRo = getNormal(x, z, x + 1, z, x, z - 1);
         midPoints.push_back(getMidPoint(x, z, x, z - 1, x + 1, z));
         normals.push_back(nRo);
      }
   }

   Vector3d sum;
   sum.x = 0;
   sum.y = 0;
   sum.z = 0;
   for(unsigned int i = 0; i < normals.size(); i++) {
      sum = sum + normals[i]; 
   }

   double sqrLength = sum.x * sum.x + sum.y * sum.y + sum.z * sum.z;
   // Calculate normFact = 1/sqrt(sqrLength)
   float normFact = (float)sqrLength;
   float normFacthalf = 0.5f*normFact;
   int i = *(int*)&normFact;
   i = 0x5f375a86 - (i>>1);
   normFact = *(float*)&i;
   normFact = normFact*(1.5f-normFacthalf*normFact*normFact);
   // normFact = normFact*(1.5f-normFacthalf*normFact*normFact); // added precision

   // Normalize.
   sum.x *= normFact;
   sum.y *= normFact;
   sum.z *= normFact;

   return sum;
}

Vector3d HeightMapClassifier::getMidPoint(int x1, int z1, int x2, int z2, int x3, int z3) const
{
   double h1 = curCells()[x1][z1].getHeight();
   double h2 = curCells()[x2][z2].getHeight();
   double h3 = curCells()[x3][z3].getHeight();
   double x1d, x2d, x3d, z1d, z2d, z3d;
   _curMap->grid2worldMM(x1d, z1d, x1, z1);
   _curMap->grid2worldMM(x2d, z2d, x2, z2);
   _curMap->grid2worldMM(x3d, z3d, x3, z3);

   Vector3d v1(x1d, h1, z1d);
   Vector3d v2(x2d, h2, z2d);
   Vector3d v3(x3d, h3, z3d);
   Vector3d sum = v1 + v2 + v3;
   
   sum.x = sum.x / 3.0;
   sum.y = sum.y / 3.0;
   sum.z = sum.z / 3.0;

   return sum;
}

Vector3d HeightMapClassifier::getNormal(int x1, int z1, int x2, int z2, int x3, int z3) const
{
   /// \todo: mal profilen ob static machen bei der oft aufgerufenen fn evtl. besser.
   double h1 = curCells()[x1][z1].getHeight();
   double h2 = curCells()[x2][z2].getHeight();
   double h3 = curCells()[x3][z3].getHeight();
   double x1d, x2d, x3d, z1d, z2d, z3d;
   _curMap->grid2worldMM(x1d, z1d, x1, z1);
   _curMap->grid2worldMM(x2d, z2d, x2, z2);
   _curMap->grid2worldMM(x3d, z3d, x3, z3);

   Vector3d a, b;
   // a = 1 -> 2
   a.x = x2d - x1d;
   a.z = z2d - z1d;
   a.y = h2 - h1;

   // b = 1 -> 3
   b.x = x3d - x1d;
   b.z = z3d - z1d;
   b.y = h3 - h1;

   // Kreuprodukt fuer Normalenvektor
   Vector3d res;
   res.x = a.y * b.z - a.z * b.y;
   res.y = a.z * b.x - a.x * b.z;
   res.z = a.x * b.y - a.y * b.x;
   
   double sqrLength = res.x * res.x + res.y * res.y + res.z * res.z;
   // Calculate x = 1/sqrt(sqrLength)
   float x = (float)sqrLength;
   float xhalf = 0.5f*x;
   int i = *(int*)&x;
   i = 0x5f375a86 - (i>>1);
   x = *(float*)&i;
   x = x*(1.5f-xhalf*x*x);
   // x = x*(1.5f-xhalf*x*x); // added precision

   // Normalize.
   res.x *= x;
   res.y *= x;
   res.z *= x;

   return res;
}

/**
 * Calculate the average pitch around x,z.
 * Assumes, that the normals have been computed already by computeNormals().
 */
void HeightMapClassifier::getAvgPitchAround(int x, int z, double & pitch, double & maxPitchDelta) const
{
   double normal[] = {0,0,0};
   if (curCells()[x][z].getMetaData().size() == 3) {
      normal[0] = curCells()[x][z].getMetaData().at(0);
      normal[1] = curCells()[x][z].getMetaData().at(1);
      normal[2] = curCells()[x][z].getMetaData().at(2);
   }
   else {
      pitch = 0.0;
      maxPitchDelta = 0.0;
      return;
   }

   double minCosDelta = HUGE_VAL;
   for(int xx = x - 1; xx <= x + 1; xx++) {
      for(int zz = z - 1; zz <= z + 1; zz++) {
         if(xx == x && zz == z)
            continue;
         if(!_curMap->pointInMap(xx, zz))
            continue;
         double dotProd = 0;
         for(int i = 0; i < 3; i++) {
            dotProd += curCells()[xx][zz].getMetaData().at(i) * normal[i];
         }
         if(dotProd < minCosDelta)     // annahme: normalen sind nicht entgegengesetzt -> dann ist abb. winkel -> cos monoton und somit kann man min cos -> max winkel direkt auf dot nehmen
            minCosDelta = dotProd;
      }
   }

   pitch = RAD2DEG(acos(normal[1]));
   maxPitchDelta = RAD2DEG(acos(minCosDelta));
}


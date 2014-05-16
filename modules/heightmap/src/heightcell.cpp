#include <math.h>
#include "heightmap/heightcell.h"
#include "utilities/timeutil.h"
#include "utilities/misc.h"
#include "utilities/paramfile.h"
#include <iostream>

#ifdef HIERARCHICAL_MAPPING
#include "hierarchicalMapping/heightCellMap.h"
#endif

using namespace std;

#define MAX_MAHALANOBIS (2.0)
#define WARN_MAHALANOBIS (0)

#define MIN_VARIANCE (50.0 * 50.0)
///< Minimum variance a cell can ever have (\f$\sigma = 1cm\f$)

HeightCell::HeightCell(double iHeight, double iVariance)
{
   _height = iHeight;
   _variance = iVariance;
   _filteredHeight = iHeight;
   _filteredVariance = iVariance;
   _lastUpdateTime.tv_sec = 0;
   _lastUpdateTime.tv_usec = 0;

   _distance = HUGE_VAL;
   _isMrfClass = false;
   _visible = false;
   _isPerimeter = false;

   _class = ELC_FLAT_GROUND; 

   //_class = ELC_NOT_INITIALIZED;

   _sigmaValid = false;
   _isCoveredBorder = false;
   _isMasked = false;
   _vegetation = 0;
   _cleared = false;
   r=g=b=0.0;
}

HeightCell::~HeightCell()
{
}
      
//RobotData* HeightCell::_robotData = NULL;

#ifdef HIERARCHICAL_MAPPING
void HeightCell::updateFromWorldPoint(const Vector3d & p, const struct timeval & now)
{
   integrateMeasurement(p.y, 100.0 * 100.0, now);  ///\todo: besseres sensormodell - evtl. curpos in scan3d mit reinpacken (6dof)
   updateFlagsAfterHeightChange();
}

/**
 * Call this function passing, the updateTime, an array for heights, classes, variances.
 *
 * \param [in] cellIndex, index of this cells params in the arrays.
 */
void HeightCell::updateFromMessage(int cellIndex, va_list ap)
{
   struct timeval updateTime = va_arg(ap, struct timeval);
   short* heights = va_arg(ap, short*);
   unsigned char* classes = va_arg(ap, unsigned char*);
   double* variances = va_arg(ap, double*);

   short height = heights[cellIndex];
   double variance = 1.0 * 1.0;
   if(variances != NULL) {
      variance = variances[cellIndex];
   }
   integrateMeasurement(height, variance, updateTime, true);

   enum ELEVATION_CLASSES cls = ELC_UNCLASSIFIED;
   if(classes != NULL) {
      cls = (enum ELEVATION_CLASSES)classes[cellIndex];
      setClass(cls);

      updateFlagsAfterHeightChange(false);
   } else {
      updateFlagsAfterHeightChange();
   }
}

void HeightCell::updateFlagsAfterHeightChange(bool setClassNotCurrent)
{
   setFlag(BinaryFlags::HEIGHT_SENT_NOT_CURRENT);
   setFlag(BinaryFlags::FILTERED_HEIGHT_NOT_CURRENT);
   setFlag(BinaryFlags::HEIGHT_NOT_CURRENT_IN_VIEWER);
   if(setClassNotCurrent) {
      setFlag(BinaryFlags::CLASSIFICATION_NOT_CURRENT);
   }

   double wx, wz;
   getCenterCoordinates(wx, wz);
   getMap()->startNeighborhoodTraversal(wx, wz);
   while(getMap()->neighborhoodTraversalHasNextCell()) {
      HeightCell & c = getMap()->neighborhoodTraversalGetNextCell();
      c.setFlag(BinaryFlags::FILTERED_HEIGHT_NOT_CURRENT);
      if(setClassNotCurrent)
         c.setFlag(BinaryFlags::CLASSIFICATION_NOT_CURRENT);
   }
   ///\todo: Wie kommen neighborpatches jetzt in _flaggedPatches??? atm gar nicht...
}
#endif

/// Uses Kalman Filter for estimation
void HeightCell::integrateMeasurement(double height, double variance, const struct timeval & now, bool directUpdate)
{
   _sigmaValid = false;

   if(directUpdate) {            // if used as container class
      _height = height;
      _variance = variance;
      _lastUpdateTime = now;
      return;
   }

   //updateErrorVariance(now);

   if(sqrMahalanobisDistance(height) > MAX_MAHALANOBIS * MAX_MAHALANOBIS) {
      if(WARN_MAHALANOBIS)
         M_INFO3("Mahalanobis Distance too big, not updating by kalman filter: M %.2f, Sigma: %.2f h %.2f h_mess %.2f, sigma: %.2f.\n", 
               mahalanobisDistance(height), sqrt(_variance), _height, height, sqrt(_variance));

      // pull measurement up (upwards wall)
      if(height > _height) {
         _height = height;
         _variance = variance;
         return;
      } else {    // measurement down, but out of mahalab. (downwards wall)
         variance *= 20;
      }
   }

   _height = (variance * _height + _variance * height) / (variance + _variance);
   _variance = (variance * _variance) / (variance + _variance);
   if(_variance < MIN_VARIANCE)
      _variance = MIN_VARIANCE;
}

// TODO
void HeightCell::updateErrorVariance(const struct timeval & now)
{
   (void) now;
   //struct timeval updateTime = now;
   //if(_robotData != NULL) {
   //  _variance += _robotData->varianceDrivenBetween(_lastUpdateTime, updateTime);
   //   _lastUpdateTime = updateTime;
   //}
}

bool HeightCell::resetOldCellHeight(const double & now, double & height)
{
   double cellAge = now - TimevalSecs(&_lastUpdateTime);
  
   if (cellAge < Params::g_maxCellAge) {
      height = _height;
      return false;
   }

   // Degrade cell
   double dir = -1.0; // drag down
   if (_height<0.0)
     dir = 1.0; // raise up

   double newHeight = _height + dir * (Params::g_heightDecrementPerUpdate * 1000.0);

   if ((dir == 1.0 && newHeight > 0.0)  || (dir == -1.0 && newHeight < 0.0) )
      newHeight = 0.0;

   _height = newHeight;
   _variance = 1000.0 * 1000.0;
   height = _height;
   return true;
}

#ifdef HIERARCHICAL_MAPPING
void HeightCell::swapWH(struct weightedHeight* w1, struct weightedHeight* w2)
{
   double weight1 = w1->weight;
   double height1 = w1->height;
   w1->weight = w2->weight;
   w1->height = w2->height;
   w2->weight = weight1;
   w2->height = height1;
}

/// Performs stupid bubblesort -> hopefully faster for this small numbers
void HeightCell::sortWeightedHeights(int num, struct weightedHeight * whs)
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

/**
 * Currently implements median filter.
 */
void HeightCell::filterCell()
{
   if(!_flags.isSet(BinaryFlags::FILTERED_HEIGHT_NOT_CURRENT))
      return;

   BaseHierarchicalMap<HeightCell>* map = getMap(); 

   // static -> no reallocations
   static struct weightedHeight weightedHeights[9];

   double newHeight = getRawHeight();
   double newVariance = getRawVariance();
   if(newHeight == 0.0)
      newVariance = 1000*1000;
   bool uninitializedCell = false;
   if((newHeight == 0) && (newVariance >= 500*500))
      uninitializedCell = true;
   int initializedNeighbors = 0;   // number of neighbor cells, that are initialized.

   double wx, wz;
   getCenterCoordinates(wx, wz);
   map->startNeighborhoodTraversal(wx, wz);

   double sumWeight = 0;
   int count = 0;
   while(map->neighborhoodTraversalHasNextCell()) {
      int dx, dz;
      HeightCell & c = map->neighborhoodTraversalGetNextCell(dx, dz);

      weightedHeights[count].weight = 1.0/c.getRawVariance();
      if((dx == 0) && (dz == 0))
         weightedHeights[count].weight *= 2.0;
      weightedHeights[count].height = c.getRawHeight();
      if(weightedHeights[count].height == 0)
         weightedHeights[count].weight = 0;
      if(weightedHeights[count].height != 0)
         initializedNeighbors++;
      else if(c.getRawVariance() < 500.0 * 500.0)
         initializedNeighbors++;
      sumWeight += weightedHeights[count].weight;
      count++;
   }

   sortWeightedHeights(count, weightedHeights);

   double sumReached = 0;
   sumWeight /= 2.0;
   for(int i = 0; i < count; i++) {
      sumReached += weightedHeights[i].weight;
      if(sumReached >= sumWeight) {
         if(!uninitializedCell || initializedNeighbors >= 1)   // was 5
            newHeight = weightedHeights[i].height;
         break;
      }
   }

   _filteredHeight = newHeight;
   _filteredVariance = newVariance;

   _flags.clearFlag(BinaryFlags::FILTERED_HEIGHT_NOT_CURRENT);
   setFlag(BinaryFlags::FILTERED_HEIGHT_SENT_NOT_CURRENT);
}
#endif

///\todo: wenn mapping w flags return smoothed, wenn smooth current. && smoothing enabled in map?
double HeightCell::getHeight() const
{
#ifdef HIERARCHICAL_MAPPING
   const BaseHierarchicalMap<HeightCell>* const map = getMap();
   if(dynamic_cast<const HeightCellMap* const>(map) == NULL)
      return _height;
   
   bool smoothingEnabled = (dynamic_cast<const HeightCellMap* const>(map))->smoothingEnabled();
   if(smoothingEnabled) ///\todo: warn wenn accessed und nicht current
      return _filteredHeight;
   else
      return _height;
#else
   return _height;
#endif
}

double HeightCell::getRawHeight() const
{
   return _height;
}
      
double HeightCell::getFilteredHeight() const
{
   return _filteredHeight;
}

double HeightCell::getSampleHeight() const
{
   return getHeight();     // these calculations are too slow, use just height; todo somehow cache sampleheight to prevent

   struct timeval t = getCurrentTimeTS();
   srand48(t.tv_usec);
   double sigma = getSigma();
   double sum = 0;
   for(int i = 0; i < 12; i++) {
      sum += drand48() * 2 * sigma - sigma;  // add [-b,b]
   }
   return sum/2 + getHeight();
}

double HeightCell::getSigma() const
{
   if(_sigmaValid)
      return getSigmaCached();
   else
      return getSigmaDirect();
}

///\todo: impl. analog getHeight
double HeightCell::getVariance() const
{
   return _variance;
}

double HeightCell::getRawVariance() const
{
   return _variance;
}

double HeightCell::getSigmaCached() const
{
   return _sigmaCached;
}

struct timeval HeightCell::getLastUpdateTime() const
{
   return _lastUpdateTime;
}

double HeightCell::getSigmaDirect() const
{
   _sigmaCached = sqrt(_variance);
   _sigmaValid = true;
   return _sigmaCached;
}

/// Fast implementation for smoothing, just float accuracy
double HeightCell::getInvSigmaFast() 
{
   float x = (float)_variance;
   float xhalf = 0.5f*x;
   int i = *(int*)&x;
   i = 0x5f375a86 - (i>>1);
   x = *(float*)&i;
   x = x*(1.5f-xhalf*x*x);
   // x = x*(1.5f-xhalf*x*x); // added precision
   return x;
}

/// The Mahalanobis distance of height to this heightcell.
/**
 *  Calculates: \f$ \sqrt{\frac{(height - \_height)^2}{\sigma^2}} \f$
 *  \param[in] height The height to measure the mahalanobis distance to.
 *  \return The mahalanobis distance to height.
 */
double HeightCell::mahalanobisDistance(double height) const
{
   return sqrt(sqrMahalanobisDistance(height));
}

double HeightCell::sqrMahalanobisDistance(double height) const
{
   return (height - _height) * (height - _height) / _variance;
}

enum HeightCell::ELEVATION_CLASSES HeightCell::getClass() const
{
   return _class;
}

/**
 * Set class only if cell is not mrf class already or mrfClass is true
 */
void HeightCell::setClass(enum ELEVATION_CLASSES cls, bool mrfClass)
{
   if(_isMrfClass && !mrfClass)
      return;
   _class = cls;
   _isMrfClass = mrfClass;
}

bool HeightCell::isMrfClassified() const
{
   return _isMrfClass;
}

vector<int> & HeightCell::getRegions() 
{
   return _regions;
}

vector<int> & HeightCell::getBorders() 
{
   return _borders;
}

vector<double> & HeightCell::getMetaData()
{
   return _metaData;
}

const vector<double> & HeightCell::getMetaData() const
{
   return _metaData;
}

bool HeightCell::isClassified() const
{
   return (_class != ELC_UNCLASSIFIED);
}

/** 
 * Offset to substract from an (int)ELEVATION_CLASSES to get a zero based offset for the true classes.
 * True classes are all the classes that have meaning for the robot, i.e. all but ELC_NOT_INITIALIZED and ELC_UNCLASSIFIED.
 */
int HeightCell::getTrueClassOffset()
{
   return 2;
}

/// The number of true classes, see getTrueClassOffset.
int HeightCell::getNumTrueClasses()
{
   return ELC_NUM_CLASSES - getTrueClassOffset();
}

/// The true (valid) class as an integer from 0 to getNumTrueClasses(), see getTrueClassOffset.
int HeightCell::getTrueClass() const
{
   return ((int)_class - getTrueClassOffset());
}


void HeightCell::writeToStream(ostream &os)
{
   os << _height << " " << _filteredHeight << " " << _variance << " " << _filteredVariance << " ";
   os << _isMrfClass << " " << _class << " ";
   os << _sigmaCached << " "; 
   os << _sigmaValid << " ";
   os << _metaData.size() << " ";
   for (unsigned int i=0; i<_metaData.size(); i++)
      os <<  _metaData[i] << " "; 
   os << endl;
}


void HeightCell::readFromStream(istream &is)
{
   is >> _height;
   is >> _filteredHeight; 
   is >> _variance; 
   is >> _filteredVariance;

   is >> _isMrfClass; 

   int dummy;
   is >> dummy;
   _class = (ELEVATION_CLASSES) dummy;
   is >> _sigmaCached; 
   is >> _sigmaValid;

   unsigned int s;
   is >> s;
   _metaData.clear();
   for (unsigned int i=0; i<s; i++) {
      double meta;
      is >>  meta;
      _metaData.push_back(meta); 
   }

   gettimeofday(&_lastUpdateTime,0);
}

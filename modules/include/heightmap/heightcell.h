#ifndef HEIGHTCELL_H
#define HEIGHTCELL_H

#include <vector>
#include <string>
#include <stdarg.h>
#include <sys/time.h>


#ifdef HIERARCHICAL_MAPPING
#include "hierarchicalMapping/baseCell.hpp"
#endif

#ifndef HIERARCHICAL_MAPPING
/// A Cell in a heightmap
class HeightCell 
#else
/// A Cell in a heightmap
class HeightCell : public BaseCell<HeightCell>
#endif
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
   void swapWH(struct weightedHeight* w1, struct weightedHeight* w2);
   void sortWeightedHeights(int num, struct weightedHeight * whs);

   public:
      HeightCell(double iHeight = 0.0, double iVariance = 1000.0 * 1000.0);
      virtual ~HeightCell();

      enum ELEVATION_CLASSES {
         ELC_UNCLASSIFIED,
         ELC_NOT_INITIALIZED,
         ELC_FLAT_GROUND,
         ELC_RAMPED_GROUND,
         ELC_DRIVABLE_OBSTACLE,
         ELC_WALL,
         ELC_STAIRS,
         ELC_NUM_CLASSES
      };

      std::string getClassName() const {
         switch (_class) {
            case ELC_UNCLASSIFIED:
               return "UNCLASSIFIED";
               break;
            case ELC_NOT_INITIALIZED:
               return "NOT:INITIALIZED";
               break;
            case ELC_FLAT_GROUND:
               return "FLAT_GROUND";
               break;
            case ELC_RAMPED_GROUND:
               return "RAMPED_GROUND";
               break;
            case ELC_DRIVABLE_OBSTACLE:
               return "DRIVABLE_OBSTACLE";
               break;
            case ELC_WALL:
               return "WALL";
               break;
            case ELC_STAIRS:
               return "STAIRS";
               break;
            default:
               return "UNKNOWN!";
               break;
         }
         return "";
      }



#ifdef HIERARCHICAL_MAPPING
      virtual void updateFromWorldPoint(const Vector3d & p, const struct timeval & now);
      virtual void updateFromMessage(int cellIndex, va_list ap);
      virtual void updateFlagsAfterHeightChange(bool setClassNotCurrent = true);
#endif

      void integrateMeasurement(double height, double variance, const struct timeval & now, bool directUpdate = false);
      bool resetOldCellHeight(const double & now, double & height);
      
      /// Save data to stream
      void writeToStream(std::ostream &os);
      void readFromStream(std::istream &os);

      /// Best height estimate for this cell.
      double getHeight() const;
      /// Raw height estimate - i.e. unfiltered.
      double getRawHeight() const;
      /// Get the filtered height.
      double getFilteredHeight() const;
      double getSampleHeight() const;
      double getSigma() const;
      double getVariance() const;
      double getRawVariance() const;
      struct timeval getLastUpdateTime() const;

      double getInvSigmaFast();

      void updateErrorVariance(const struct timeval & now);

#ifdef HIERARCHICAL_MAPPING
      /// Calculates filteredHeight based on neighbor cells.
      void filterCell();
#endif

      double mahalanobisDistance(double height) const;
      double sqrMahalanobisDistance(double height) const;

      enum ELEVATION_CLASSES getClass() const;
      void setClass(enum ELEVATION_CLASSES cls, bool mrfClass = false);
      bool isMrfClassified() const;

      bool isPerimeter() const {return _isPerimeter;}
      void togglePerimeter()  {_isPerimeter=!_isPerimeter;}
      void togglePerimeterOn()  {_isPerimeter=true;}
      void togglePerimeterOff()  {_isPerimeter=false;}
      bool isClassified() const;
      bool isVisible() const {return _visible;}
      void setVisible(bool v)  {_visible=v;}
      bool isCleared() const {return _cleared;}
      void setCleared(bool v)  {_cleared=v;}
      int getTrueClass() const;
      static int getTrueClassOffset();
      static int getNumTrueClasses();
      std::vector<double> & getMetaData();
      const std::vector<double> & getMetaData() const;
      std::vector<int> & getRegions();
      std::vector<int> & getBorders();
      void setIsCoveredBorder(bool val) {_isCoveredBorder=val;}
      bool getIsCoveredBorder() {return _isCoveredBorder;}

      void setIsMasked(bool val) {_isMasked=val;}
      bool getIsMasked() {return _isMasked;}

      void setVegetation(int val) {_vegetation=val;}
      int getVegetation() {return _vegetation;}

      double getDistance() {return _distance;}
      void setDistance(double d) {_distance=d;}

      void setRGB(double _r, double _g, double _b) {r=_r;g=_g;b=_b;}
      void getRGB(double &_r, double &_g, double &_b) {_r=r;_g=g;_b=b;}
      bool isBorderCell() const {return _borders.size() > 0;}

   protected:
      double getSigmaCached() const;
      double getSigmaDirect() const;

   protected:
      double _height;      // mm
      double _filteredHeight; ///< Height, filtered considering neighbor cells.
      double _variance;    // when sampled from multiple points
      double _filteredVariance;
      double _distance;
      struct timeval _lastUpdateTime;

      bool _isMrfClass;    ///< was _class set by MRF or simple?
      enum ELEVATION_CLASSES _class;

      std::vector<double> _metaData;  ///< Added data to the heightcell
      std::vector<int> _regions;  ///< Added data to the heightcell
      std::vector<int> _borders;  ///< Added data to the heightcell


      mutable double _sigmaCached;
      mutable bool _sigmaValid;
      bool _visible;
      bool _isPerimeter;
      bool _cleared;
      bool _isCoveredBorder;
      bool _isMasked;
      char  _vegetation;
      double r,g,b;
};

#endif


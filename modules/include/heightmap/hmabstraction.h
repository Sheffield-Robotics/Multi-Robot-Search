#ifndef HM_ABSTRACTION
#define HM_ABSTRACTION

#include <map>
#include <vector>
#include "heightmap/heightmap.h"
#include "heightmap/visibility.h"
#include "visibility_helpers.h"

class HmAbstraction {
    public:
       struct Tile {
          Tile() : truemax(0), max(0), valid(false), closed(false), visSize(0), pos(0,0), level(-1) {};
          Tile(double _min, double _max, Visibility::Pos _pos, int _level) : truemax(_min), max(_max), valid(true), closed(false), visSize(0), pos(_pos), level(_level) {};

         bool operator()(const Tile t1, const Tile t2) const
         {
            if (t1.visSize > t2.visSize)
               return true;
            else
               return false;
         }

          double truemax, max;
          bool valid;
          bool closed;
          int visSize;
          Visibility::Pos pos;
          int level;
       };

       // Construction
       HmAbstraction(HeightMap* h) : _hmap(h) {};
       ~HmAbstraction() {};

       void generateMaxDetectionSet(double pH, double pR, double tH, deque<Visibility::Pos> &result);

       HeightMap* getLevelHM(int level);
       //HeightMap* getSubMapHM(int level, Visibility::Pos& pos);

       bool writeLevelHM(string filename, int level);

    private:
       void allocateLevel(int level);
       void generateLevel(int level);
       void writeHeightMapPPM(string filename, HeightMap* hm);
       void rankVisibility(deque<Visibility::Pos> selection, HeightMap* hm, int level, double pH, double pR, double tH, std::multimap < int, Visibility::Pos> &result);
       void filterOutClosedCells(Visibility::VisSet& vset, int level );
       void writeClosedPPM(string filename, int level);

       bool tileOnLevel(int level, int x, int y) {
          if (_abstractions.find(level) == _abstractions.end()) {
             printf("LEVEL NOT FOUND %d\n",level);
             return false;
          }
          else if (x<0 || y<0 || x>=(int)_abstractions[level].size() || y>=(int)_abstractions[level][0].size()) {
             printf("NOT IN MAP %d %d of %d %d \n",x,y,(int)_abstractions[level].size(), (int)_abstractions[level][0].size());
             return false;
          }
          else
             return true;
       }

       void writeTestPPM(string filename, HeightMap* hmap);
       
    private:
       HeightMap* _hmap;
       std::map < int, std::vector< std::vector <Tile> > > _abstractions;

};

#endif



#ifndef VISIBILITY_H
#define VISIBILITY_H


#include <map>
#include <deque>
#include <list>
#include <set>
#include "heightmap/heightmap.h"
#include "heightmap/heightcell.h"
#include "utilities/misc.h"
#include "utilities/math/bresenham.h"

using std::deque;
using std::set;

#define DEBUG_LINE_TRAJECTORIES 1

class HeightMap;

class Visibility {
	friend class Viewer;
	
   public:
      struct Pos{
         Pos(int _x, int _y) : x(_x), y(_y) {};
         Pos() : x(0), y(0) {};

         bool operator()(const Pos p1, const Pos p2) const
         {
            if (p1.x > p2.x)
               return true;
            else if (p1.x < p2.x)
               return false;
            else if (p1.y > p2.y)
               return true;
            else
               return false;
         }

		void print() {printf("(%d,%d)",x,y);}
        double sqrdist(Pos& o) {return (o.x-x)*(o.x-x)+(o.y-y)*(o.y-y);};
        double dist(Pos& o) {return sqrt(sqrdist(o));};

        int x,y;
      };

     struct Vertex {
         Vertex(int _id, Pos _pos, double _height=0.0, double _range = 0.0) : id(_id), pos(_pos), height(_height), range(_range) {};
         int id;
         Pos pos;
         double height;
         double range;
         deque<deque<Pos> > polyset;
      };
      struct Edge {
         Edge(int _id1, int _id2, bool _shady) : id1(_id1), id2(_id2), shady(_shady) {};
         Edge() : id1(-1), id2(-1), shady(false) {};
         int id1,id2;
         bool shady;

         bool operator()(const Edge e1, const Edge e2) const
         {
            if (e1.id1 > e2.id1)
               return true;
            else if (e1.id1 < e2.id1)
               return false;
            else if (e1.id2 > e2.id2)
               return true;
            else
               return false;
         }
    };

      typedef std::map<Pos,bool,Pos> VisSet;
      typedef std::map<Pos,VisSet,Pos> VisSetIndex;

      Visibility(HeightMap* _map, double _pursuerHeight, double _pursuerRange, double _targetHeight); 
      void reset();
      int computeVisibilitySet(int x, int y, VisSet& vset);
      int computeVisibilitySet(int nodeId, VisSet& vset);
      
      int filterOutAssignedCells(VisSet& vset);
      
      bool loadGraph(string filename);
      bool saveGraph(string filename);
      void eraseMarkings(bool visibility, bool cleared);
      void setVisiblilityMarkings(VisSet &vset, bool erase);
      void tesCircle(int radius, int x0, int y0);
      void testingWriteAllBorderComponentsToPPM(char* filename);

      double getPursuerHeight() {return pursuerHeight;}
      double getPursuerRange() {return pursuerRange;}
      double getTargetHeight() {return targetHeight;}

      HeightMap* getHeightMap() {return hmap;};

      void markNodeVisSetAsClear(int nodeId, bool erase);

	  bool computeRegionSet(string base_filename, int num_span_trees, bool bias_spanning);
	  void computePerimeterSchedule();
      void detectRegionBorder(int x, int y, bool traversability);

      int get_visibility_line_cost(int x1, int y1, int x2, int y2, 
          std::list<Pos>& uav_positions);
      
      int get_max_steps();
      
   private:
       
       void get_max_coverage_pose_on_line(int x_e, int y_e, 
           int x2, int y2,
           int& x_uav_max, int& y_uav_max, 
           VisSet& covered_poses, bool first_pos, int last_uav_x = 0,int last_uav_y = 0);
       
       double get_visible_line(int x1, int y1, int x2, int y2, int& x_end, int& y_end, VisSet& covered_poses, VisSet& covering_poses, bool& broken, bool forward);
      double get_coverage_score( int x_uav, int y_uav, int x_e, int y_e, int x2, int y2, VisSet& covered_poses );
      
      void get_coverage_poses(int x_uav, int y_uav, int& x_e, int& y_e, int x2, int y2, VisSet& covered_poses );
      
      int computeVisibleLinePoints(int x1, int y1, int x2, int y2, VisSet &vset, bool record_hidden = false);
      void setVisible(int x, int y, bool v);
      void setCleared(int x, int y, bool v);
      int posToIndex(const Pos& p);
      void computeCircle(int radius, int x0, int y0, deque<Pos> &arc);
      void computeDistanceMap();
      void writeDistancePGM(string filename);
      void writeDistancePPM(string filename);
      void writeClassesPPM(string filename);
      vector<Visibility::Pos> getAllBorderCells(int x, int y, int border);
      bool verify(int x, int y,bool silent=false);
      double getHeight(int x, int y); 
      void writeExperimentInfo(string fname);

      Visibility::Pos drawRandomFromE();
      void generateE();

   private:
      HeightMap* hmap;
      VisSetIndex visSetIdx;
      set<Pos,Pos> E;
      string _filename;
      double targetHeight;
      double pursuerHeight;
      double pursuerRange;
      std::map <Pos,Visibility::VisSet,Pos> _visibilityCache;

   public:
      deque<Vertex> vertices;
      set<Edge,Edge> edges;
};

inline bool operator==(const Visibility::Edge& e1, const Visibility::Edge& e2) 
{
   return (e1.id1 == e2.id1 && e1.id2 == e2.id2);
}

inline bool operator==(const Visibility::Pos& p1, const Visibility::Pos& p2) 
{
   return (p1.x == p2.x && p1.y == p2.y);
}

std::ostream & operator << (std::ostream& o, const Visibility::Pos& p);
std::ostream & operator << (std::ostream& o, const Visibility::Vertex& v);
std::ostream & operator << (std::ostream& o, const Visibility::Edge& e);

#endif

#ifndef _ANIMATOR_H
#define _ANIMATOR_H

#include <deque>
#include <map>

#include "heightmap/visibility.h"
#include "heightmap/heightmap.h"
#include "planner/planningMap.h"
#include "planner/aStarSearch.h"
#include "heightmap/geomap.h"

using std::deque;

class Animator {
   public:
      struct Pursuer {
         Pursuer() : id(0), currVert(Visibility::Vertex(0, Visibility::Pos(0,0))), lastVert(Visibility::Vertex(0, Visibility::Pos(0,0))), plan(NULL), isMoving(false), isFree(true)  {};
         Pursuer(int _id, Visibility::Vertex p) : id(_id), currVert(p), lastVert(Visibility::Vertex(0, Visibility::Pos(0,0))), plan(NULL), isMoving(false), isFree(false) {};

         bool operator()(const Pursuer p1, const Pursuer p2) const
         {
            if (p1.id > p2.id)
               return true;
            else if (p1.id < p2.id)
               return false;
            return false;
         }

         void moveTo(Visibility::Vertex& vert) {lastVert = currVert; currVert=vert;isMoving=true;isFree=false;}
         void setFree() {lastVert = currVert; isMoving=false;isFree=true; if(plan!=NULL) delete plan;plan=NULL;}

         int id;
         Visibility::Vertex currVert;
         Visibility::Vertex lastVert;
         deque<PlanNode>* plan;
         bool isMoving;
         bool isFree;
      };
      struct AColor {
         AColor(double _r, double _g, double _b) : r(_r), g(_g), b(_b) {};

         AColor& operator= (const AColor& p) {
            r=p.r;g=p.g;b=p.b;
            return *this;
         }

            
         double r,g,b;
      };

      Animator(Visibility *visibility, HeightMap *map);
      bool init(string filename, bool loadExistingSchedule);
      void draw();
      bool update(string justWriteToFileName="");
      unsigned int numPursuer() {return _pursuer.size();}
      void initPursuerList(double wx, double wy);
      int getMaxSteps() {return _maxStep;};
      void setGeoMapPointer(GeoMap* g) {_geo=g;}

   private:
      bool loadCompressedSchedule(string filename);
      bool loadFlatSchedule(string filename);
      void drawAgent(const Pursuer& pursuer);
      AColor getAgentColor(int id);
      int idx_to_agentId(int idx);
      int nodeId_to_idx(int id);
      void computeSimpleAssignment(vector<Pursuer*>& agents, vector<Visibility::Vertex*>& targets, int result[]);
      void computeHungarianAssignment(vector<Pursuer*>& agents, vector<Visibility::Vertex*>& targets, int result[]);
      void writeHeaderToExecFile(string fname, int numAgents, int numVerices, int numSteps);
      void writeAgentPathToExecFile(string fname, int step, Pursuer& p, HeightMap* map, double exexMinutes);
      double getExecutionMeters(int planLength);
      double getExecutionMinutes(int planLength);

   private:
      Visibility* _vis;
      HeightMap* _map;
      PlanningMap* _pmap;
      int _currStep;
      int _maxStep;
      int _numAgents;
      bool _initialized;
      double lastStepTime;
      deque<Pursuer> _pursuer;
      vector<vector<int> > _schedule;
      vector<vector<int> > _compressedScheduleNewVertices;
      vector<vector<int> > _compressedScheduleFreeVertices;
      double _startX, _startY;
      GeoMap* _geo;
      double _totalPlanLength;
};

std::ostream & operator << (std::ostream& o, const Animator::Pursuer& p);

inline bool operator==(const Animator::Pursuer& p1, const Animator::Pursuer& p2) 
{
   return (p1.id == p2.id);
}

#endif

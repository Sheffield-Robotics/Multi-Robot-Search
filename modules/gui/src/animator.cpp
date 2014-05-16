#include "gui/animator.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include "utilities/misc.h"
#include "gui/glHelpers.h"
#include "planner/hungarian.h"
#include "utilities/gpsconvert.h"
#include "utilities/paramfile.h"

#define DEBUG_LOAD 0
#define DEBUG_HUNGARIAN 0
#define HUNAM_WALKING_SPEED (1.1) // 1.1ms = 4km/h // im m/s

using namespace std;

Animator::Animator(Visibility *visibility, HeightMap* map)
{
   _vis = visibility;
   _map = map;

   // Create planning map
   _pmap = NULL;
   _currStep = 0;
   _initialized = false;
   lastStepTime = getCurrentTime();
   _maxStep = 0;
   _startX=_startY=0;
   _numAgents = 0;
   _geo = NULL;
}

bool Animator::init(string filename, bool loadExistingSchedule)
{
   bool ret = false;
   if (loadExistingSchedule) {
      if (Params::g_use_compressed_strategy) {
         ret = loadCompressedSchedule(filename);

         printf("Compressed Schedule loaded:\n");
         printf("---------------------------\n");
         for (int i=0; i<_maxStep; i++) {
            printf("Step %d    NEW ",i);
            for (unsigned int j=0; j<_compressedScheduleNewVertices[i].size(); j++)
               printf("%d ",_compressedScheduleNewVertices[i][j]);
            printf(" ---> FREE ");
            for (unsigned int j=0; j<_compressedScheduleFreeVertices[i].size(); j++)
               printf("%d ",_compressedScheduleFreeVertices[i][j]);
            printf("\n");
         }
      }
      else
         ret = loadFlatSchedule(filename);
      M_INFO2("Num agents: %d Length: %d\n",_numAgents, _maxStep);
   }
   else {
      _numAgents = 0;
      _maxStep = 0;
      _pursuer.clear();
      _schedule.clear();
      _compressedScheduleNewVertices.clear();
      _compressedScheduleFreeVertices.clear();
   }

   // Decide start locations
   //////////////////////////a
   double wx = Params::g_start_world_x;
   double wy = Params::g_start_world_y;
   initPursuerList(wx,wy);

   // Reset agents
   for (unsigned int i=0; i<_pursuer.size(); i++) {
      _pursuer[i].currVert.pos = Visibility::Pos(_startX,_startY);
      _pursuer[i].lastVert.pos = Visibility::Pos(_startX,_startY);
      _pursuer[i].setFree();
   }

   // Reset marking
   _vis->eraseMarkings(true, true);

   _currStep = 0;
   _initialized = ret;
   return ret;
}


bool Animator::loadCompressedSchedule(string filename)
{
   // open file for reading
   ifstream file (filename.c_str(), ios::in);
   if (!file.is_open()) {
      M_ERR("Error: could not open file: %s for reading",filename.c_str());
      return false;
   }
   M_INFO2("Reading compressed schedule from file '%s'\n",filename.c_str());

   string line;
   istringstream inStream;

   // Read number of agents
   _numAgents = 0;
   getline(file, line);
   if (line.find('#') == string::npos || line.size() < 1) {
      M_ERR("First line should contain the number of agents like: '# num'. Couldn't find in line '%s'\n",line.c_str());
      file.close();
      return false;
   }
   else {
      inStream.str(line);
      char d;
      inStream >> d;
      inStream >> _numAgents; 
      if (_numAgents>0);
      else {
         M_ERR("Negative number of agents.\n");
         file.close();
         return false;
      }
   }

   // count lines
   _maxStep = 0;
   while (!file.eof()) {
      getline (file, line);
      if (line.size()>1)
         _maxStep++;
   }
   _maxStep /=2;
   file.close();
   if (_maxStep < 1) {
      M_ERR("Compressed schedule file possibly corrupted!\n");
      _numAgents=0;
      _maxStep=0;
      return false;
   }
   file.open(filename.c_str(), ios::in);
   getline (file, line); // read out num agents

   // Read-in the schedule
   _compressedScheduleFreeVertices.resize(_maxStep);
   _compressedScheduleNewVertices.resize(_maxStep);
   int step = 0;

   _compressedScheduleNewVertices[0].clear();
   while (!file.eof() && step<_maxStep) 
   {
      getline (file, line);
      if (line.size()<1)
         continue;

      if (line.find("new") != string::npos) {
         line = line.substr(4,line.size());
         inStream.clear();
         inStream.str(line);
         _compressedScheduleNewVertices[step].clear();
         while (inStream.good()) {
            int node=-1;
            inStream >> node;
            if (node>=0) {
               _compressedScheduleNewVertices[step].push_back(node);
            }
         }
      }
      else {
         M_ERR("Error parsing file!");
         return false;
      }

      getline (file, line);
      if(line.find("freed") != string::npos) {
         line = line.substr(6,line.size());
         inStream.clear();
         inStream.str(line);
         _compressedScheduleFreeVertices[step].clear();
         while (inStream.good()) {
            int node=-1;
            inStream >> node;
            if (node>=0) {
               _compressedScheduleFreeVertices[step].push_back(node);
            }
         }
      }
      else {
         M_ERR("Error parsing file!");
         return false;
      }
      step++;
   }
   file.close();

   return true;
}


bool Animator::loadFlatSchedule(string filename)
{
   // open file for reading
   ifstream file (filename.c_str(), ios::in);
   if (!file.is_open()) {
      M_ERR("Error: could not open file: %s for reading",filename.c_str());
      return false;
   }
   M_INFO2("Reading schedule from file '%s'\n",filename.c_str());

   string line;
   istringstream inStream;

   // Read number of agents
   _numAgents = 0;
   getline(file, line);
   if (line.find('#') == string::npos || line.size() < 1) {
      M_ERR("First line should contain the number of agents like: '# num'. Couldn't find in line '%s'\n",line.c_str());
      file.close();
      return false;
   }
   else {
      inStream.str(line);
      char d;
      inStream >> d;
      inStream >> _numAgents; 
      if (_numAgents>0);
      else {
         M_ERR("Negative number of agents.\n");
         file.close();
         return false;
      }
   }

   // count lines
   _maxStep = 0;
   while (!file.eof()) {
      getline (file, line);
      if (line.find("STRATEGY") != string::npos)
         break;
      if (line.size()>1)
         _maxStep++;
   }
   file.close();
   file.open(filename.c_str(), ios::in);
   getline (file, line); // read out num agents

   // Read-in the schedule
   _schedule.resize(_maxStep);
   for (int i=0; i< _maxStep; i++)
      _schedule[i].resize(_numAgents);
   int step = 0;
   while (!file.eof() && step<_maxStep) {
      getline (file, line);
      if (line.size()<1)
         continue;
      inStream.clear();
      inStream.str(line);

      for (int i=0; i<_numAgents; i++) {
         int node=-1;
         inStream >> node;
         _schedule[step][i] = node;
      }
      step++;
   }
   file.close();
   return true;
}


bool Animator::update(string justWriteToFileName)
{
   if (_maxStep == 0) {
      _totalPlanLength = 0;
      M_WARN("No schedule for simulation\n");
      return false;
   }

   if (justWriteToFileName != "" && _currStep==0) 
      writeHeaderToExecFile(justWriteToFileName, _pursuer.size(), _vis->vertices.size(), _maxStep);

   // Reset when end or start is reached
   if (_currStep >= _maxStep) {
      _vis->eraseMarkings(true, true);
      _currStep = 0;
      _totalPlanLength = 0;
      for (unsigned int i=0; i<_pursuer.size(); i++) 
      {
         _pursuer[i].currVert.pos = Visibility::Pos(_startX,_startY);
         _pursuer[i].setFree();
      }
   }
   else if (_currStep < 0) {
      _currStep = 0;
      _totalPlanLength = 0;
      for (unsigned int i=0; i<_pursuer.size(); i++) 
      {
         _pursuer[i].currVert.pos = Visibility::Pos(_startX,_startY);
         _pursuer[i].setFree();
      }
   }

   M_INFO3("Step %d of %d\n",_currStep+1, _maxStep);
   if (Params::g_show_visibility_during_schedule && justWriteToFileName=="")
      _vis->eraseMarkings(true, false);

   // Determine list of available agents and targets 
   // for this step.
   vector<Pursuer*> agents;
   vector<Visibility::Vertex*> targets;

   if (Params::g_use_compressed_strategy)
   {
      // Free all agents according to schedule
      vector<int> freed_nodeIds;
      if (_currStep > 0) {
         for (unsigned int i=0; i<_compressedScheduleFreeVertices[_currStep-1].size(); i++) 
         freed_nodeIds.push_back(_compressedScheduleFreeVertices[_currStep-1][i]);
      }
      for (unsigned int i=0; i<freed_nodeIds.size(); i++) {
         int nodeId = freed_nodeIds[i];

         // Mark area as clear
         _vis->markNodeVisSetAsClear(nodeId, false);

         // Find agent on node
         int foundIdx = -1;
         for (unsigned int j=0; j<_pursuer.size(); j++)  {
            if (_pursuer[j].currVert.pos == _vis->vertices[nodeId_to_idx(nodeId)].pos) {
               foundIdx = j;
               _pursuer[j].setFree();
               break;
            }
         }
         if (foundIdx != -1) 
            M_INFO1("Freed agent %d from node %d\n", idx_to_agentId(foundIdx), nodeId);
         else
            M_ERR("ERROR: Cannot find agent for freed node %d\n", nodeId);
      }

      // Add all free agents
      for (unsigned int i=0; i<_pursuer.size(); i++) {
         if (_pursuer[i].isFree) 
            agents.push_back(&_pursuer[i]);
      }
      printf("Added %d free agents\n",(int)agents.size());

     // Add targets
      for (unsigned int i=0; i < _compressedScheduleNewVertices[_currStep].size(); i++) {
         // find target
         int t = (int)_compressedScheduleNewVertices[_currStep][i];
         bool found = false;
         for (unsigned int j=0; j < _vis->vertices.size(); j++) {
            if (_vis->vertices[j].id == t) {
               targets.push_back(&_vis->vertices[j]);
               found = true;
            }
         }
         if (found) 
            M_INFO1("Adding target %d from scheduled nodes \n", targets.back()->id);
         else
            M_ERR("ERROR: Cannot find target with ID %d\n", t);
      }
   }
   else {
      M_ERR("Flat schedule not implemented yet!\n");
      return false;
   }

   printf("Avail. Agents (%d): ",(int) agents.size());
   for (unsigned int i=0; i<agents.size(); i++) 
      cout << *agents[i] ;
   cout << endl;

   printf("Avail. Targets (%d): ",(int) targets.size());
   for (unsigned int i=0; i<targets.size(); i++) 
      cout << *targets[i] ;
   cout << endl;

   // Compute Assignment
   ///////////////////////////
   int assign[agents.size()];
   //computeSimpleAssignment(agents, targets, assign);
   computeHungarianAssignment(agents, targets, assign);

   // Move agents according to assignment
   for (unsigned int i=0; i<agents.size(); i++) {
      Pursuer* pursuer = agents[i];
      if (assign[i] >= 0) {
         Visibility::Vertex* target = targets[assign[i]];
         pursuer->moveTo(*target);
         M_INFO2("Assign agent %d to target %d\n",pursuer->id, target->id);
      }
      else {
         M_INFO2("Agent %d has no assignment\n",pursuer->id);
      }
   }

   // Update plans
   bool changed = false;
   int maxLength = 0;
   if (Params::g_draw_agent_trajectories) {
      // Create planning map
      if (_pmap == NULL) {
         _pmap = new PlanningMap();
         _pmap->createFromHeightMap(_map);
      }
      AStarSearch astar;
      for (unsigned int j=0; j<_pursuer.size(); j++)  {
         if (_pursuer[j].plan != NULL)
            delete _pursuer[j].plan;
            _pursuer[j].plan = NULL;
         // Compute path
         double planlength = -1.0;   
         if ( _pursuer[j].isMoving) {
            Visibility::Pos start = _pursuer[j].lastVert.pos;
            Visibility::Pos goal = _pursuer[j].currVert.pos;
            _pursuer[j].plan = astar.search(_pmap, start.x, start.y, goal.x, goal.y);
            if (_pursuer[j].plan != NULL) {
               planlength = (double) _pursuer[j].plan->size();
               if ((int) _pursuer[j].plan->size() > maxLength)
                  maxLength = _pursuer[j].plan->size();
               if ((int) _pursuer[j].plan->size() == 0)
                  M_ERR("Plan length is Zero!\n");
            }
            changed = true;
         }
         if (justWriteToFileName != "") 
            writeAgentPathToExecFile(justWriteToFileName, _currStep, _pursuer[j], _map, getExecutionMinutes(planlength));
      }
   }
   
   M_INFO1("Step execution time: %1.2lf min and length: %1.2lf meter (%1.2lf miles)\n",
         getExecutionMinutes(maxLength), getExecutionMeters(maxLength), getExecutionMeters(maxLength) / 1600.0);
   _totalPlanLength += maxLength;

   // Compute and mark visibility
   if (Params::g_show_visibility_during_schedule && justWriteToFileName == "") {
      for (unsigned int j=0; j<_pursuer.size(); j++)  {
         //int nodeId = _pursuer[j].currVert.id;
         if (_pursuer[j].isFree)
            continue;
         Visibility::VisSet vs;
         _vis->computeVisibilitySet(_pursuer[j].currVert.pos.x, _pursuer[j].currVert.pos.y, vs);
         _vis->setVisiblilityMarkings(vs, false);
         changed = true;
      }
   }

   // Set all agents as not moving
   for (unsigned int j=0; j<_pursuer.size(); j++)  
      _pursuer[j].isMoving = false;

   _currStep++;
   if (_currStep == _maxStep) {
      M_INFO1("End of plan. Approx. time: %1.2lf min and Plan length: %1.2lf meter (%1.2lf miles)\n",
            getExecutionMinutes(_totalPlanLength), getExecutionMeters(_totalPlanLength),getExecutionMeters(_totalPlanLength) / 1600.0);
      _totalPlanLength=0;
   }
 
   return changed;
}

void Animator::draw()
{
   if (!_initialized)
      return;

   for (unsigned int i=0; i<_pursuer.size(); i++) {
      drawAgent(_pursuer[i]);
   }
}

void Animator::computeSimpleAssignment(vector<Pursuer*>& agents, vector<Visibility::Vertex*>& targets, int result[])
{   
   // Grab for each agent the first target
   for (unsigned int i=0; i<agents.size(); i++) {
      if (i>=targets.size())
         result[i] = -1; // These agents have nothing to do
      else
         result[i] = i;
   }
}

void Animator::computeHungarianAssignment(vector<Pursuer*>& agents, vector<Visibility::Vertex*>& targets, int result[])
{
   if (DEBUG_HUNGARIAN)
      printf("HUNGARIAN:   NUM AGENTS: %d     NUM TARGETS: %d\n",(int) agents.size(), (int)targets.size());

   // Compute cost matrix
   int columns = targets.size();
   int rows = agents.size();
   int ** costs = new int*[columns];
   for (int i=0; i<columns; i++)
      costs[i] = new int [rows];
   for (int i=0; i<columns; i++) {
      for (int j=0; j<rows; j++) {
         Visibility::Pos tPos = targets[i]->pos;
         Visibility::Pos aPos = agents[j]->currVert.pos;
         double d = tPos.sqrdist(aPos);
         costs[i][j] = (int) d;
      }
   }

   // Compute assignment
   hungarian_problem_t hp;
   hungarian_init(&hp, costs , columns, rows, HUNGARIAN_MODE_MINIMIZE_COST);
   hungarian_solve(&hp);
   if (DEBUG_HUNGARIAN) {
      fprintf(stderr, "costs:");
      hungarian_print_costmatrix(&hp);
   }
   hungarian_solve(&hp);
   if (DEBUG_HUNGARIAN) {
      fprintf(stderr, "assignment:");
      hungarian_print_assignment(&hp);
   }

   for (unsigned int i=0; i<agents.size(); i++)
      result[i] = -1; 
   for (int i=0; i<columns; i++) {
      int row = hungarian_get_assignment(&hp, i);
      if (DEBUG_HUNGARIAN) 
         printf("agent %d --> target %d\n", row, i);
      result[row] = i;
   }

   // Free memory
   hungarian_free(&hp);
   for (int i=0; i<columns; i++)
      delete [] costs[i];
   delete [] costs;
}

void Animator::drawAgent(const Pursuer& pursuer)
{
  if (_map == NULL )
      return;

   // Select color
  AColor col(0,0,0);
  if (Params::g_use_multiple_agent_colors)
     col = getAgentColor(pursuer.id);
  else {
     if (pursuer.isFree)
        col = AColor(0, 0, 0);
     else
        col = AColor(0.5, 0.5, 0.5);
  }
  glDisable(GL_LIGHTING);

  // Translate and rotate model global
  double g_tx, g_ty, g_tz;
  int cx = pursuer.currVert.pos.x;
  int cy = pursuer.currVert.pos.y;
  _map->grid2world(g_tx, g_ty, cx, cy);
  g_tz = _map->getCellsMM()[cx][cy].getHeight()/ 1000.0;

  // Box and sphere
  double s=1.0;
  if (Params::g_draw_large_agents)
     s = std::min(_map->sizeX(), _map->sizeZ()) * _map->resolution() / 100.0;
  double width = 1.0 * s;
  double height = 3.0 * s;
  double rad = 1.0 * s;

  // Agent indicator
  glColor3f(col.r,col.g,col.b);
  glLineWidth(3.0);
  double hh2 = g_tz + height*2*10;
  double ss2 = s*10.0;
  drawGoalMarker(g_tx, g_ty, hh2, ss2); 
  glBegin(GL_LINES);
  glVertex3f(g_tx, g_ty, g_tz);
  glVertex3f(g_tx, g_ty, hh2);
  glEnd();

  glPushMatrix();
  glTranslatef(g_tx,g_ty,g_tz+height/2.0);
  //glPointSize(2);
  double fak = 1.0;
  if (pursuer.isFree && Params::g_use_multiple_agent_colors)
     fak = 3.0;
  drawBox(width,width,height);
  glTranslatef(0,0,height/2+0.8*rad*fak);
  drawSphere(rad*fak);
  glPopMatrix();

  glEnable(GL_LIGHTING);

  // Draw plan
  double offset = 1 + pursuer.id * 0.1;
  if (pursuer.plan != NULL)
     drawPlan(_map, pursuer.plan, offset);
}

Animator::AColor Animator::getAgentColor(int id)
{
   int col = (id % 12);
   switch(col) {
      case 0:
         return AColor(0.5, 0.5 ,0.5);
      case 1:
         return AColor(0.0 ,1.0, 0.0);
      case 2:
         return AColor(0.0 ,0.0, 1.0);
      case 3:
         return AColor(1.0, 1.0, 1.0);
      case 4:
         return AColor(1.0, 0.68, 0.0);
      case 5:
         return AColor(1.0, 1.0, 0.0);
      case 6:
         return AColor(1.0, 0.0, 1.0);
      case 7:
         return AColor(0.0, 1.0, 1.0);
      case 8:
         return AColor(0.5, 1.0, 0.0);
      case 9:
         return AColor(0.5, 1.0, 0.5);
      case 10:
         return AColor(0.5, 0.0, 0.5);
      case 11:
         return AColor(0.0, 0.0, 0.5);
      default:
         M_ERR("Color error\n");
         break;
   }
   return AColor(0,0,0);
}

void Animator::initPursuerList(double wx, double wy)
{
   int cx=0,cy=0;
   _map->world2grid(wx, wy, cx, cy);

   if (!_map->pointInMap(cx, cy)) {
      M_ERR("Start location (%1.2lf, %1.2lf) --> (%d, %d) is not on map! Using 0,0 instead\n", wx,wy, cx,cy);
      cx=0;cy=0;
   }
   _startX = cx;
   _startY = cy;

   _pursuer.clear();

   for (int i=0; i<_numAgents; i++) {
      _pursuer.push_back(Pursuer(i, Visibility::Vertex(-1,Visibility::Pos(_startX,_startY))));
      _pursuer.back().setFree();
   }
   M_INFO1("Pursuer start location: %lf %lf (%lf %lf)\n",wx, wy, _startX, _startY); 
}

int Animator::nodeId_to_idx(int id)
{
   for (unsigned int i=0; i<_vis->vertices.size(); i++)
      if (_vis->vertices[i].id==id)
         return i;
   M_ERR("Cannot find node with id %d!!!\n",id);
   return -1;
}

int Animator::idx_to_agentId(int idx)
{
   if (idx<0 || idx >= (int)_pursuer.size()) {
      M_ERR("No agent at idx %d!!\n",idx);
      return -1;
   }
   else
      return _pursuer[idx].id;
}

std::ostream & operator << (std::ostream& o, const Animator::Pursuer& p)
{
   o << " [" << " id:" << p.id << " currVert: " << p.currVert << " lastVert: " << p.lastVert << " is moving:" << p.isMoving << " is free:" << p.isFree << "] ";
   return o;
}

void Animator::writeHeaderToExecFile(string fname, int numAgents, int numVerices, int numSteps)
{
   if (_geo == NULL) {
      M_ERR("Need geomap pointer first!\n");
      return;
   }

   ofstream outFile;
   outFile.open(fname.c_str(),ios::out);
   outFile << "# Execution File v1.0\n";
   outFile << "# File: " << fname.c_str() << endl;
   outFile << "# Num agents: " << numAgents << endl;
   outFile << "# Num vertices: " << numVerices << endl;
   outFile << "# Num steps: " << numSteps << endl;
   outFile << "# LINE FORMAT: step agentId isFree isMoving executionMinutes goalVertexIndex planSize plan<LAT1 LON1 LAT2 LON2 ... LATplanSize,LONplanSize>" << endl;
   outFile.close();
}

void Animator::writeAgentPathToExecFile(string fname, int step, Pursuer& p, HeightMap* map, double exexMinutes)
{
   if (_geo == NULL) {
      M_ERR("Need geomap pointer first!\n");
      return;
   }

   // Generate plan
   vector <double> longitute;
   vector <double> latitude;
   if (p.plan != NULL) {
      int lastX=0, lastY=0;
      for (unsigned int i=1; i<p.plan->size(); i++) {
         PlanNode n = (*p.plan)[i];
         int gridX= (int) n.x;
         int gridY= (int) n.z;
         double worldX=0, worldY=0;

         map->grid2world(worldX, worldY, gridX, gridY);
         //HACK
         //worldX = n.x;
         //worldY = n.z;
         if (i>=1 && i != (p.plan->size() -1) && Params::g_min_dist_gps_poses > 0) {
            double d = hypot((worldX-lastX), (worldY-lastY));
            if (d<Params::g_min_dist_gps_poses) 
               continue;
         }
         double east = worldX;
         double north = -worldY;
         double lat=0,lon=0;
         _geo->image2world(&east, &north);
         char UTMZone[100] = GLOBAL_UTM_ZONE;
         gps_UTMtoLL(north, east, UTMZone, &lat, &lon);
         //printf("Plan %d %d  World: %lf %lf  East/North: %lf %lf Lon/Lat: %lf %lf\n", 
         //      gridX, gridY, worldX, worldY, east, north, lat, lon); 
         longitute.push_back(lon);
         latitude.push_back(lat);
         lastX = worldX;
         lastY = worldY;
      }
   }

   if (p.isMoving && longitute.size() == 0) {
      M_ERR("Agent is moving but has no plan! Putting goal as waypoint\n");
      int gridX= (int) p.currVert.pos.x;
      int gridY= (int) p.currVert.pos.y;
      double worldX=0, worldY=0;
      map->grid2world(worldX, worldY, gridX, gridY);
      double east = worldX;
      double north = -worldY;
      double lat=0,lon=0;
      _geo->image2world(&east, &north);
      char UTMZone[100] = GLOBAL_UTM_ZONE;
      gps_UTMtoLL(north, east, UTMZone, &lat, &lon);
      longitute.push_back(lon);
      latitude.push_back(lat);
   }

   // Write stuff to file
   int path_length = longitute.size();
   ofstream outFile;
   outFile.open(fname.c_str(),ios_base::app);
   outFile << step << " " << p.id << " " << p.isFree << " " << p.isMoving << " " << exexMinutes << " " << p.currVert.id << " "<< path_length << " ";

   for (int i=0; i<path_length; i++)
      outFile << setiosflags(ios::fixed) << setprecision(10) << latitude[i] << " " << longitute[i] << " ";
   outFile << endl;
   outFile.close();
}

double Animator::getExecutionMinutes(int planLength)
{
   if (planLength<0)
      return -1.0;
   double length = (double) planLength * _map->resolution();
   return (length / HUNAM_WALKING_SPEED) / 60.0;
}

double Animator::getExecutionMeters(int planLength)
{
   return (double) planLength * _map->resolution();
}


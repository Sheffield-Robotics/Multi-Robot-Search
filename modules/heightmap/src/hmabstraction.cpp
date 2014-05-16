#include "heightmap/hmabstraction.h"
#include "utilities/timing.h"
#include "planner/planningMap.h"

#define DEBUG_WRITE_PPMS (0)

using namespace std;

void HmAbstraction::generateLevel(int level)
{
   if (level < 2) {
      M_ERR("Level 1 must be generated from hm directly\n");
      return;
   }

   if (_abstractions.find(1) == _abstractions.end()) {
      M_ERR("Level 1 does not exists!\n");
      return;
   }

   int step = pow(2.0,level-1);
   int sx = (int)((double)_abstractions[1].size() / (double)step);
   int sy = (int)((double)_abstractions[1][0].size() / (double)step);

   M_DEBUG("Generating abstraction level %d size: (%d, %d) step: %d\n", level, sx, sy,step);

   allocateLevel(level);
   int needNCellsForNegValue = ceil( pow((double)step,2) / Params::g_neg_value_fraction ); //(step^2); // step^2 is all cells, 
   for (int x=0; x<sx; x++) {
      for (int y=0; y<sy; y++) {
         double truemax = -HUGE_VAL;
         double max = -HUGE_VAL;
         int count = 0;
         int countClosedCells = 0;
         for (int xx = x*step; xx < (x+1)*step; xx++) {
            for (int yy = y*step; yy < (y+1)*step; yy++) {
               
               if ( xx >= _hmap->sizeX() || yy >= _hmap->sizeZ() )
               {
                  M_ERR("Critical error invalid memory at %d %d from %d %d",xx,yy,x,y);
                  break;
               }
               
               // This should not happen
               if (!tileOnLevel(1, xx, yy)) 
                  continue;

               // Compute statistics
               double v = _abstractions[1][xx][yy].max;
               if (v>max) max=v;

               if(!_abstractions[1][xx][yy].closed)
               {
                  count++;
                  if (v>truemax) 
                     truemax=v;
               }
               else 
               {
                  ++countClosedCells;
               }          
               // Ignore closed points
            }
         }
         _abstractions[level][x][y] = Tile(truemax,max,Visibility::Pos(x,y),level);
         // count in open cells
         if (countClosedCells >= needNCellsForNegValue ) {
            _abstractions[level][x][y].closed = true;
            truemax = 0.0;
         }
         else
         {
            _abstractions[level][x][y].closed = false;
         }
      }
   }
}

void HmAbstraction::allocateLevel(int level)
{
   if (level <= 0) {
      M_ERR("Level 0 should not be unused!\n");
      return;
   }

   int step = pow(2.0,level-1);
   int sx = (int)((double)_hmap->sizeX() / (double)step);
   int sy = (int)((double)_hmap->sizeZ() / (double)step);

   vector<Tile> tt(sy);
   vector< vector <Tile> > abstr(sx, tt);
   _abstractions[level] = abstr;

   M_DEBUG("Allocated level %d size: %d %d\n",level, sx,sy);
}

HeightMap* HmAbstraction::getLevelHM(int level)
{
   if (_abstractions.find(level) == _abstractions.end()) {
      M_ERR("Abstraction level %d not allocated!\n",level);
      return NULL;
   }

   int sx = _abstractions[level].size();
   int sy = _abstractions[level][0].size();
   int scalingFactor = pow(2.0,(level-1));
   double res = _hmap->resolution() * scalingFactor;
   //cout << " getLevelHM res=" << res << " sx " << sx << " sy " << sy << endl;
   //cout << " scaling " << scalingFactor << " " << endl;
   HeightMap *map = new HeightMap(res, 0, 0, sx, sy);

   // scalingFactor*2 are all cells in neighborhood
   int needNCellsForNegValue = ceil( pow((double)scalingFactor,2)/ Params::g_neg_value_fraction );//(scalingFactor^2); // step^2 is all cells, 
   // if needNCellsForNegValue == 1, then then one shrub makes all shrub
   // if needNCellsForNegValue == (scalingFactor^2) then all shrub make one shrub
   
   struct timeval t;
   gettimeofday(&t,0);
   double var = 50.0 * 50.0;
   for (int x=0; x<sx; x++) {
      for (int y=0; y<sy; y++) {
         double value = _abstractions[level][x][y].max;
         map->getCellsMM()[x][y].integrateMeasurement(value, var, t, true);
         if ( map->getCellsMM()[x][y].getHeight() != value )
            cout << "LOOK" << endl;
         int hasShrub = 0;
         int isNonTraversable = 0;
         for (int xx = x*scalingFactor; xx < (x+1)*scalingFactor; xx++) {
            for (int yy = y*scalingFactor; yy < (y+1)*scalingFactor; yy++) {
               if ( xx >= _hmap->sizeX() || yy >= _hmap->sizeZ() )
               {
                  M_ERR("Critical error invalid memory at %d %d from %d %d",xx,yy,x,y);
                  break;
               }
               
               if ( _hmap->getCellsMM()[xx][yy].getVegetation() > 0 )
                  ++hasShrub;
               if ( _hmap->getCellsMM()[xx][yy].getClass() != HeightCell::ELC_FLAT_GROUND )
                  ++isNonTraversable;
            }
         }
         if ( hasShrub >= needNCellsForNegValue )
            map->getCellsMM()[x][y].setVegetation( 1 );
         else
            map->getCellsMM()[x][y].setVegetation( 0 );
         if ( isNonTraversable >= needNCellsForNegValue )
            map->getCellsMM()[x][y].setClass( HeightCell::ELC_WALL );
         else
            map->getCellsMM()[x][y].setClass( HeightCell::ELC_FLAT_GROUND );
      }
   }
   return map;
}

void HmAbstraction::generateMaxDetectionSet(double pH, double pR, double tH, deque<Visibility::Pos> &result)
{
   result.clear();
   
   if (DEBUG_WRITE_PPMS) {
      writeHeightMapPPM("/tmp/REALheightmap.ppm",_hmap);
   }
   // Create planning map for closing cells nearby obstacles
   PlanningMap* pmap = new PlanningMap();
   pmap->createFromHeightMap(_hmap);
   
   // Generate bottom layer directly from height map
   allocateLevel(1);
   int numOpenCells = 0;
   for (int x=0; x<_hmap->sizeX(); x++) {
      for (int y=0; y<_hmap->sizeZ(); y++) {
         double value = _hmap->getCellsMM()[x][y].getHeight();
         _abstractions[1][x][y] = Tile(value, value,Visibility::Pos(x,y),1);
         if ((_hmap->getCellsMM()[x][y].getClass() != HeightCell::ELC_FLAT_GROUND) || // cell is on non-flat ground
               (!pmap->isTraversable(x, y, Params::g_skillExploration)) || // cell is non-traversable by planner
               (Params::g_use_shrubs &&  _hmap->getCellsMM()[x][y].getVegetation() > 0) // cell is covered with shrubs
            )
            _abstractions[1][x][y].closed = true;
         else {
            _abstractions[1][x][y].closed = false;
            numOpenCells++;
         }
      }
   }

   int totalCells = numOpenCells;

   // Create abstraction layers
   int k = 1;
   double ratio = 1.0;
   while (ratio < 8.0) {
      k++;
      generateLevel(k);
      ratio = (double) _hmap->sizeX() / (double) _abstractions[k].size();

   }

   // FOR DEBUGGING WRITE HIGHTMAPS
   if (DEBUG_WRITE_PPMS) {
      for (int i=1; i<=k; i++) {
         char name1[100];
         char name2[100];
         sprintf(name1,"/tmp/level_%d.ppm",i); 
         sprintf(name2,"/tmp/closed_%d.ppm",i); 
         writeLevelHM(string(name1),i);
         writeClosedPPM(string(name2),i);
      }
   }
   //int countWhileLoops = 0;
   if (DEBUG_WRITE_PPMS) {
      writeTestPPM("/tmp/mapreal.ppm", _hmap );
   }
   while (1) {
      // Select best tile from top map
      deque<Visibility::Pos> selection;
      for (unsigned int x=0; x<_abstractions[k].size(); x++) {
         for (unsigned int y=0; y<_abstractions[k][0].size(); y++) {
            if (!_abstractions[k][x][y].closed)
               selection.push_back(Visibility::Pos(x,y));
         }
      }
      if (selection.empty() || numOpenCells < Params::g_imporved_sampling_min_area_size) {
         M_INFO3("FINISHED with %d nodes, %d\n",result.size(),numOpenCells);
         break;
      }
      else {
         M_INFO2("Still %d to go --> %1.2lf percent. Selection: %d. Have %d nodes\n",
               numOpenCells, 100.0 * (double) numOpenCells / (double) totalCells, selection.size(), result.size());
      }

      HeightMap* hm = getLevelHM(k);
      multimap < int, Visibility::Pos> rank;
      rankVisibility(selection, hm, k, pH, pR, tH, rank);
      delete hm;

      Visibility::Pos best = (*rank.rbegin()).second;
      double percent = 100.0 * ((double)(*rank.rbegin()).first / (double)(_abstractions[k].size()*_abstractions[k][0].size()));
      M_INFO3("Top Level %d: Best Tile found: %d %d percent: %1.1lf\n",k, best.x,best.y,percent);
      
      // REDUCE THE ABSTRACTIONS!
      //if ( (*rank.rbegin()).first * _hmap->resolution() <  Params::g_imporved_sampling_min_area_size )
      if ( (*rank.rbegin()).first < 20 )
      {
         // the best visibility set has to be of stable size, at least 20 pixel, or else this level is not appropriate
         M_INFO3("REDUCING ABSTRACTION LEVEL\n");
         --k;
         M_INFO3("Reached level %d\n",k);
         if ( k == 1 )
         {
            M_INFO3("Reached level 1  - breaking \n");
            break;
         }
         continue;
      }
      
      // Search for best tile until bottom
      int curr_level = k;
      int bestSize = 0;
      bool found = true;
      if (DEBUG_WRITE_PPMS) {
         char name[100];
         sprintf(name,"/tmp/closed_step_%d_level_%d.ppm",int(result.size()), int(curr_level)); 
         writeClosedPPM(name,curr_level);
      }
      
      while (curr_level-- > 1) 
      {
         if (DEBUG_WRITE_PPMS) {
            char name[100];
            sprintf(name,"/tmp/closed_step_%d_level_%d.ppm",int(result.size()), int(curr_level)); 
            writeClosedPPM(name,curr_level);
         }
         
         HeightMap* hm = getLevelHM(curr_level);
         deque<Visibility::Pos> selection2;
         // Possible TODO Use the best N best peaks
         int step = Params::g_improved_sampling_down_level_search_width;
         for (int x=-step; x<step; x++) {
            for (int y=-step; y<step; y++) {
               int nx = best.x*2 + x;
               int ny = best.y*2 + y;
               int sx = _abstractions[curr_level].size();
               int sy = _abstractions[curr_level][0].size();
               if (nx<0 || ny<0 || nx>=sx || ny>=sy)
                  M_ERR("Out of bounds: l%d %d %d \n",curr_level,nx,ny);
               else if (!_abstractions[curr_level][nx][ny].closed)
                     selection2.push_back(Visibility::Pos(nx,ny));
            }
         }
         if (selection2.empty()) {
            delete hm;
            found = false;
            M_INFO3("selection2 empty - breaking\n");
            break;
         }
         M_INFO3("selection2 size %d \n",selection2.size() );
         
         multimap < int, Visibility::Pos> rank2;
         //if ( curr_level == 1 )
         //   rankVisibility(selection2, _hmap, curr_level, pH, pR, tH, rank2);
         //else
         rankVisibility(selection2, hm, curr_level, pH, pR, tH, rank2);
         //char name1[100];
         //sprintf(name1,"/tmp/hmap_lvl%d_step%d.ppm",curr_level,result.size()); 
         //writeTestPPM(string(name1), hm);
         delete hm;
         
         best = (*rank2.rbegin()).second;
         int size = (*rank2.rbegin()).first;
         bestSize = size;
         double percent = 100.0 * ((double)size / (double)(_abstractions[curr_level].size()*_abstractions[curr_level][0].size()));

         M_INFO3("Level %d: Best Tile found: %d %d  with visset %d percent: %1.1lf\n",curr_level, best.x, best.y, size,percent);
      }

      if (!found) {
         //Visibility::Pos failed = (*rank.rbegin()).second;
         break;
      }
      else if (_abstractions[1][best.x][best.y].closed == true)
         M_ERR("Selected closed node !!!\n");

      Visibility vis(_hmap, pH, pR, tH); 
      Visibility::VisSet vset;
      vis.computeVisibilitySet(best.x, best.y, vset);
      
      if ( Params::g_use_only_one_region_per_pixel )
      {
         filterOutClosedCells(vset,1);

         if (DEBUG_WRITE_PPMS) {
            char name1[100];
            sprintf(name1,"/tmp/vSet_VpreFilter%d_.bmp", (int)result.size()); 
            if ( vset.size() * _hmap->resolution() > Params::g_imporved_sampling_min_area_size )
               writeVisSetToFile( string(name1), vset );
         }
         filterVisibilitySet(vset, true, best.x,best.y);
      }
      vis.setVisiblilityMarkings(vset, true);
      
      if (DEBUG_WRITE_PPMS) {
         char name1[100];
         sprintf(name1,"/tmp/vSet_V%d_.bmp",(int)result.size()); 
         if ( vset.size() * _hmap->resolution() > Params::g_imporved_sampling_min_area_size )
            writeVisSetToFile( string(name1), vset );
      }
      
      if ( vset.size() != (unsigned int) bestSize )
         cout << " ERROR vis.size not equal to bestSize" << endl;


      /// Invalidate cells
      int count = 0;
      printf("Closing cells from  %d %d\n",best.x, best.y);
      for (Visibility::VisSet::iterator it=vset.begin(); it != vset.end(); it++)  {
         Visibility::Pos p = it->first;
         if(!_abstractions[1][p.x][p.y].closed) {
            numOpenCells--;
            count++;
            _abstractions[1][p.x][p.y].closed = true;
         }
      }
      M_INFO1("Closed %d cells\n",count);
      M_INFO1("Closing best x best y\n");
      _abstractions[1][best.x][best.y].closed = true;

      // Add node to result if closing area is large enough
      
      if (count * _hmap->resolution() > Params::g_imporved_sampling_min_area_size) 
      {
         M_INFO1("Vertex good enough, add to results - covers %f need %f \n",count * _hmap->resolution(), Params::g_imporved_sampling_min_area_size);
         result.push_back(best);
      }
      else
         M_ERR("WARNING: Leaving this area uncovered since it is too small\n");

      // Recompute abstraction layers
      for (int i=2; i<=k; i++) 
         generateLevel(i);

   }

   if (DEBUG_WRITE_PPMS) {
      char name[100];
      sprintf(name,"/tmp/final_closed.ppm"); 
      writeClosedPPM(name,1);
   }

   if (numOpenCells<=0) {
      delete pmap;
      return;
   }

   // Close all holes
   M_INFO1("Closing %d left cells ...", numOpenCells);
   fflush(stdout);
   //Visibility *vis = new Visibility(_hmap, pH, pR, tH); 
   //vis->eraseMarkings(true, true);
   //for (unsigned int ii=0; ii<result.size(); ii++) {
   //   Visibility::VisSet vset;
   //   vis->computeVisibilitySet(result[ii].x, result[ii].y, vset,false);
   //   vis->setVisiblilityMarkings(vset, false);
   //}
   //delete vis;
   set<Visibility::Pos,Visibility::Pos> E;
   for (int xx=0; xx<_hmap->sizeX(); xx++) {
      for (int yy=0; yy<_hmap->sizeZ(); yy++) { 
         if ( ! _abstractions[1][xx][yy].closed ) {
            E.insert(Visibility::Pos(xx,yy));
         }
      }
   }
   M_INFO1("...done numOpenCells %d E.size = %d \n", numOpenCells, E.size() );

   if (DEBUG_WRITE_PPMS) {
      int sx = _hmap->sizeX();
      int sy = _hmap->sizeZ();
      M_INFO3("Saving HM PPM of size %d,%d \n",sx, sy);
      ofstream os;
      os.open("LEFTOVER.ppm");
      os << "P5" << endl;
      os << sy << " " << sx << " " << 255 << endl;
      for(int x=0; x<sx; x++) {
         for(int y=0; y<sy; y++) {
            int value = 0;
            if ((_hmap->getCellsMM()[x][y].getClass() == HeightCell::ELC_FLAT_GROUND) &&
                  (pmap->isTraversable(x, y, Params::g_skillExploration)) &&
                  (!Params::g_use_shrubs ||  _hmap->getCellsMM()[x][y].getVegetation() == 0) &&
                  !_hmap->getCellsMM()[x][y].isVisible()) 
               value = 255;
            char c = (unsigned char) value;
            os.put(c);
         }
      }
      os.close();
   }

   Visibility final_vis(_hmap, pH, pR, tH); 
   while (!E.empty()) {
      // Draw random cell
      int pos = (int) (drand48() * (E.size()-1));
      set<Visibility::Pos,Visibility::Pos>::iterator it = E.begin();
      while (pos-- > 0)
         it++;
      Visibility::Pos p = *it;
      E.erase(it);

      // Generate visibility set
      Visibility::VisSet vset;
      final_vis.computeVisibilitySet(p.x, p.y, vset);
      if ( Params::g_use_only_one_region_per_pixel )
      {
         filterOutClosedCells(vset,1);
         if (DEBUG_WRITE_PPMS) {
            char name1[100];
            sprintf(name1,"/tmp/vSet_VpreFilter%d_.bmp",(int)result.size()); 
            if ( vset.size() * _hmap->resolution() > Params::g_imporved_sampling_min_area_size )
               writeVisSetToFile( string(name1), vset );
         }
         
         if ( vset.size() * _hmap->resolution() > Params::g_imporved_sampling_min_area_size )
            filterVisibilitySet(vset, true, p.x,p.y);
      }

      // Remove vis set from E
      for (Visibility::VisSet::iterator it=vset.begin(); it != vset.end(); it++)  {
         if(E.find(it->first) != E.end()) {
            E.erase(it->first);
         }
      }

      // Add node to result if closing area is large enough
      if (vset.size() * _hmap->resolution() > Params::g_imporved_sampling_min_area_size) 
      {
         if (DEBUG_WRITE_PPMS) {
            char name1[100];
            sprintf(name1,"/tmp/vSet_V%d_.bmp",(int)result.size()); 
            cout << " Writing Vertex " << result.size() << endl;
            writeVisSetToFile( string(name1), vset );
         }
         
         result.push_back(p);
         for (Visibility::VisSet::iterator it=vset.begin(); it != vset.end(); it++)  {
            Visibility::Pos p = it->first;
            if(!_abstractions[1][p.x][p.y].closed) {
               _abstractions[1][p.x][p.y].closed = true;
            }
         }
      }
      else
         M_ERR("WARNING: Leaving %d cells uncovered since are is too small\n",vset.size());

      M_INFO2("Still %d to go --> %1.2lf percent. Have %d nodes\n",
            (int)E.size(), 100.0 * (double) E.size() / (double) totalCells, result.size());
   }
   delete pmap;
}

void HmAbstraction::rankVisibility(deque<Visibility::Pos> selection, HeightMap* hm, int level, double pH, double pR, double tH, multimap < int, Visibility::Pos> &result)
{
   result.clear();
   M_INFO3("Computing %d selected visibilities for size (%d, %d) res %lf pH=%lf pR=%lf tH=%lf\n",selection.size(), hm->sizeX(), hm->sizeZ(), hm->resolution(), pH, pR, tH);
   for (unsigned int i=0; i<selection.size(); i++) 
   {
      // adjust pursuer height according to true height, considering the abstraction
      int x = selection[i].x;
      int y = selection[i].y;
      double pursuerHeight = pH - (_abstractions[level][x][y].max - _abstractions[level][x][y].truemax)/1000;
      Visibility::VisSet vset;
      Visibility vis(hm, pursuerHeight, pR, tH); 
      vis.computeVisibilitySet(selection[i].x, selection[i].y, vset);
      if ( Params::g_use_only_one_region_per_pixel )
      {
         //cout << " pursuerHeight " << pursuerHeight << " pR=" << pR << " " << " tH " << tH << endl;
         //cout << " computeVisibilitySet size" << vset.size() << " lvl=" << level << endl;
         filterOutClosedCells(vset,level);
         //cout << " filterOutClosedCells size" << vset.size() << endl;
         if ( level <= 2 )
            filterVisibilitySet(vset,true, selection[i].x, selection[i].y );
         //cout << " filterVisibilitySet size" << vset.size() << endl;
         
      }
      
      vis.setVisiblilityMarkings(vset, true);
      
      pair<int, Visibility::Pos> aux(vset.size(), Visibility::Pos(selection[i].x,selection[i].y));
      result.insert(aux);
   }
}

void HmAbstraction::filterOutClosedCells(Visibility::VisSet& vset, int level )
{
   // remove all points that are closed in the abstraction
   for (Visibility::VisSet::iterator it=vset.begin(); it != vset.end(); )  
   {
      Visibility::Pos p = it->first;
      it++;
      if ( _abstractions[level][p.x][p.y].closed )
         vset.erase(p);
   }
}

bool HmAbstraction::writeLevelHM(string filename, int level)
{
   HeightMap* hm = getLevelHM(level);
   if (hm == NULL) {
      M_ERR("Abstraction level %d does not exist!\n",level);
      return false;
   }
   writeHeightMapPPM(filename, hm);
   delete hm;
   return true;
}

void HmAbstraction::writeHeightMapPPM(string filename, HeightMap* hm)
{
   int sx = hm->sizeX();
   int sy = hm->sizeZ();

   M_INFO3("Saving HM PPM of size %d,%d to %s\n",sx, sy, filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P5" << endl;
   os << sy << " " << sx << " " << 255 << endl;

   // Find max
   double max = 0.0;
   double min = HUGE_VAL;
   for(int x=0; x<sx; x++) {
      for(int y=0; y<sy; y++) {
         double value = hm->getCellsMM()[x][y].getHeight();
         if (value>max)
            max=value;
         if (value<min)
            min=value;
      }
   }
   //M_DEBUG("MAX Height IS %lf\n",max);
   //M_DEBUG("MIN Height IS %lf\n",min);
   for(int x=0; x<sx; x++) {
      for(int y=0; y<sy; y++) {
         double value = hm->getCellsMM()[x][y].getHeight();
         double f = 255.0 * ((value - min) / (max-min));
         char c = (unsigned char) f;
         os.put(c);
      }
   }
   os.close();
}

void HmAbstraction::writeTestPPM(string filename, HeightMap* hmap )
{
   enum { 
   PPM_WHITE = 0xFFFFFF00,
    PPM_BLACK = 0x00000000,
    PPM_RED = 0xFF000000,
    PPM_GREEN = 0x00FF0000,
    PPM_BLUE = 0x0000FF00,
    PPM_YELLOW = 0xFFFF0000,
    PPM_MAGENTA = 0xFF00FF00,
    PPM_CYAN = 0x00FFFF00};
   
   printf("Save Colored Distance PPM to %s\n",filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P3" << endl;
   os << hmap->sizeX() << " " << hmap->sizeZ() << endl;
   os << 255 << endl;

   // Assign colors
   map<double,int> index;
   const int MAX_COL = 1000;
   double c[MAX_COL][3];
   index[0]=0;
   c[0][0] = 255.0;
   c[0][1] = 255.0;
   c[0][2] = 255.0;

   //int count = 1;
   double maxHeight = -HUGE_VAL;
   double minHeight = HUGE_VAL;
   for(int z = 0; z < hmap->sizeZ(); z++) {
      for(int x = 0; x < hmap->sizeX(); x++) {
         double d = hmap->getCellsMM()[x][z].getHeight();
         if ( d > maxHeight )
            maxHeight = d;
         if ( d < minHeight )
            minHeight = d;
      }
   }
   
   M_DEBUG("MAX Height IS %lf\n",maxHeight);
   M_DEBUG("MIN Height IS %lf\n",minHeight);
   int foundShrub= 0;
   
   unsigned int pixel;
   
   for(int z = 0; z < hmap->sizeZ(); z++) {
      for(int x = 0; x < hmap->sizeX(); x++) {
         double d = hmap->getCellsMM()[x][z].getHeight();
         bool isShrub;
         if (hmap->getCellsMM()[x][z].getVegetation() > 0 )
            isShrub = true;
         else
            isShrub = false;
         bool isTraversable;
         if (hmap->getCellsMM()[x][z].getClass() == HeightCell::ELC_FLAT_GROUND )
            isTraversable = true;
         else
            isTraversable = false;
         double f = 255.0 * ((d - minHeight) / (maxHeight-minHeight));
         unsigned char r = (unsigned char) f;
         unsigned char g = (unsigned char) f;
         unsigned char b = (unsigned char) f;
         if ( isShrub ) 
         { 
            ++foundShrub;
            r = (unsigned char) 0.0; g = (unsigned char) 255.0; b = (unsigned char) 0.0; 
         }
         else if ( !isTraversable ) 
         { 
            r = (unsigned char) 255.0; g = (unsigned char) 0.0; b = (unsigned char) 0.0; 
         }
         pixel = (r << 24) | (g << 16) | (b << 8);
         //unsigned char rr = (PPM_RED & pixel) >> 24;
         //unsigned char gg = (PPM_GREEN & pixel) >> 16;
         //unsigned char bb = (PPM_BLUE & pixel) >> 8;
         //os << ((char*)&rr) << " ";
         //os << ((char*)&gg) << " ";
         //os << ((char*)&bb);
         if ( x > 0 )
            os << " ";
         os << (unsigned int) r << " "; os << (unsigned int) g << " "; os << (unsigned int)b;
      }
      os << endl;
   }
   os.close();
   M_DEBUG("Found %d Shrubs ", foundShrub);
}

void HmAbstraction::writeClosedPPM(string filename, int level)
{
   int sx = _abstractions[level].size();
   int sy = _abstractions[level][0].size();;

   M_INFO3("Saving Closed PPM of size %d,%d to %s\n",sx, sy, filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P5" << endl;
   os << sy << " " << sx << " " << 255 << endl;

   for(int x=0; x<sx; x++) {
      for(int y=0; y<sy; y++) {
         double f = 255.0;
         if (_abstractions[level][x][y].closed)
            f = 0.0;
         char c = (unsigned char) f;
         os.put(c);
      }
   }
   os.close();
}
      


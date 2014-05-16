#include "heightmap/visibility.h"
#include <iostream>
#include <sstream>
#include <vector>
#include "utilities/misc.h"
#include "utilities/filesysTools.h"
#include "heightmap/hmabstraction.h"
#include <algorithm>
#define DEBUG_LOAD (0)
#define DEBUG_WRITE_PPMS (0)

// This is a hack to remove lonely
// vertices. It should only be activated 
// for testing since it might happen then
// that larger areas are uncovered!
#define REMOVE_LONELY_VERTICES (1)

using namespace std;

// Constructor
Visibility::Visibility(HeightMap* _map, double _pursuerHeight, double _pursuerRange, double _targetHeight)
{
   // all in [m]
   pursuerHeight = _pursuerHeight;
   pursuerRange = _pursuerRange;
   targetHeight = _targetHeight;
   if (_map == NULL) {
      M_ERR("Got empty map pointer!\n");
      return;
   }
   hmap = _map;
}

void Visibility::reset()
{
   visSetIdx.clear();
   M_INFO1("Cleared visibility index\n");
   _visibilityCache.clear();
   M_INFO1("Cleared visibility cache\n");
}

int Visibility::computeVisibleLinePoints(int x1, int y1, int x2, int y2, VisSet &vset, bool record_hidden)
{
   int count = 0;
   double h1 = getHeight(x1,y1) + pursuerHeight;
   double maxSlope = -HUGE_VAL;
   double wx1,wy1;

   long int maxRangeSteps = pursuerRange / hmap->resolution();
   //M_WARN("MAX STEPS RAY SEARCH %d",maxRangeSteps);

   hmap->grid2worldMM(wx1,wy1,x1,y1);
   long int steps = 0;
   bresenham_param_t line;
   get_bresenham_parameters(x1, y1, x2, y2, &line);
   while (get_next_point(&line)) 
   {
      if (steps++>maxRangeSteps)
         break;
      int x,y;
      get_current_point(&line, &x, &y);
      if (!verify(x,y,true))
         break;
      double wx,wy;
      hmap->grid2worldMM(wx,wy,x,y);
      double h2 = getHeight(x,y);

     // Simple HACK
      if (Params::g_use_shrubs &&  hmap->getCellsMM()[x][y].getVegetation() > 0) {
         h2 += Params::g_shrub_height;
      }

      double d = hypot(wx-wx1, wy-wy1);
      double m = (h2 - h1) / d;

       double m_t = ((h2 + targetHeight) - h1) / d;

      if (m_t < maxSlope) {
         // Cell is hidden
         if ( record_hidden ) {
             vset[Pos(x,y)] = false;
         }
      }
      else {
         //printf("(m,max) %lf %lf (wx,wy) %lf %lf (h1,h2) %lf %lf\n",m, maxSlope, wx,wy, h1, h2);

         // Only add traversable cells 
         if (hmap->getCellsMM()[x][y].getClass() == HeightCell::ELC_FLAT_GROUND) {
            if (!Params::g_use_shrubs ||  hmap->getCellsMM()[x][y].getVegetation() <= 0) {
               count++;
               vset[Pos(x,y)] = true;
            }
         }
      }
      if (m >= maxSlope) {
         maxSlope = m;
      }
   } 
   return count;
}

void Visibility::eraseMarkings(bool visibility, bool cleared)
{
   for (int xx=0; xx<hmap->sizeX(); xx++) {
      for (int yy=0; yy<hmap->sizeZ(); yy++) {
         if (visibility)
            setVisible(xx,yy,false);
         if (cleared)
            setCleared(xx,yy,false);
      }
   }
}

void Visibility::setVisiblilityMarkings(VisSet &vset, bool erase)
{
   // Make all cells invisible
   if (erase) 
      eraseMarkings(true, false);

   for (VisSet::iterator it=vset.begin(); it != vset.end(); it++)  {
      Pos p = it->first;
      setVisible(p.x,p.y,true);
   }
}

int Visibility::computeVisibilitySet(int nodeId, VisSet& vset) 
{
   int found = -1;
   for (unsigned int j=0; j<vertices.size(); j++) { 
      if (vertices[j].id == nodeId) {
         found = j;
         break;
      }
   }
   if (found < 0) {
      M_ERR("Cannot find nodeID %d\n",nodeId);
      return -1;
   }
   return computeVisibilitySet(vertices[found].pos.x, vertices[found].pos.y,vset);
}

int Visibility::get_max_steps() {
    int maxRangeSteps = pursuerRange / hmap->resolution();
    maxRangeSteps = std::min(maxRangeSteps, hmap->sizeX());
    maxRangeSteps = std::min(maxRangeSteps, hmap->sizeZ());
    return maxRangeSteps;
}

/*
 * computes the number of robots needed to cover the line from x1,y1 to x2,y2
 *
 * Author: Andreas Kolling ( Thu Nov  1 10:59:45 CET 2012 ) 
 */
int Visibility::get_visibility_line_cost(int x1, int y1, int x2, int y2, 
std::list<Pos>& uav_positions)
{
    if ( DEBUG_LINE_TRAJECTORIES >= 1 ) {
        M_INFO1("get_visibility_line_cost (%d,%d) - (%d,%d)\n",x1,y1,x2,y2);
    }
    
    int x_uav_max = 0, y_uav_max = 0;
    int last_x_uav_max = 0, last_y_uav_max = 0;
    int x_e = x1, y_e = y1; // initialize x_e,y_e, to the starting point
    int x_e_inv = x2, y_e_inv = y2; // and x_e_inv, y_e_inv to inv point    
    VisSet covered_poses;
    bool all_points_covered = false;
    int x_e_pre = -1, y_e_pre = -1;
    //int x_e_inv_pre = -1, y_e_inv_pre = -1;
    bool first_pos = true;
    while( !all_points_covered ) {    
        x_e_pre = x_e; y_e_pre = y_e;
        get_max_coverage_pose_on_line(x_e,y_e,x2,y2, 
            x_uav_max, y_uav_max, covered_poses, first_pos, 
            last_x_uav_max, last_y_uav_max);
        first_pos = false;
        if ( verify(x_uav_max, y_uav_max,true) ) {
            get_coverage_poses( x_uav_max, y_uav_max, x_e, y_e, x2, y2,
                covered_poses); //!!!updates x_e,y_e!!!
            uav_positions.push_back( Pos(x_uav_max,y_uav_max) );
        }
        
        if ( DEBUG_LINE_TRAJECTORIES > 3 ) {
            M_INFO2("  **__**__ have %d uav poses \n ", uav_positions.size() );
            M_INFO2("  **__**__ new end point %d %d \n ", x_e, y_e );
            M_INFO2("  **__**__ Covered points %d \n ", covered_poses.size() );
        }
        
        if ( (x_e == x_e_inv && y_e == y_e_inv )
          || (x_e_pre == x_e && y_e_pre == y_e) 
          || (x_e == -1 || y_e == -1 )) {
            all_points_covered = true;
        }
        last_x_uav_max = x_uav_max;
        last_y_uav_max = y_uav_max;
    }
    
    if ( DEBUG_LINE_TRAJECTORIES > 3 ) {
        M_INFO2("___ ALL POINTS COVERED %d\n", covered_poses.size() );
        std::list<Pos>::iterator it = uav_positions.begin();
        printf(" UAV grid poses: ");
        while( it != uav_positions.end() ) {
            printf(" (%d,%d), ", (*it).x,(*it).y);
            it++;
        }
        printf(" \n ");
    }
    
    if ( DEBUG_LINE_TRAJECTORIES >= 1 ) {
        M_INFO1("get_visibility_line_cost return %d \n",uav_positions.size());
    }
    return uav_positions.size();
}

/*
 * sample one more UAV location between at x_e,y_e 
 * by traversing the line from x_e,y_e to x_2,y_2
 * 
 * Author: Andreas Kolling ( Thu Nov  1 12:37:22 CET 2012 ) 
 */
void Visibility::get_max_coverage_pose_on_line(int x_e, int y_e, int x2, int y2, 
  int& x_uav_max, int& y_uav_max, VisSet& covered_poses, bool first_pos,
    int last_uav_x, int last_uav_y) {
    
    if ( DEBUG_LINE_TRAJECTORIES > 2) {
        M_INFO2("get_max_coverage_pose_on_line ");
        M_INFO2("(%d,%d)--(%d,%d)\n",x_e,y_e,x2,y2);
    }
    
    double maxRangeStepsSqr = pow(double(get_max_steps()),2);    
    x_uav_max = -1; y_uav_max = -1;
    int x_uav, y_uav;
    double n_cells_max = 0;
    Pos end_pos(x_e,y_e);
    Pos end_pos2(x2,y2);
    Pos last_uav_pos(last_uav_x,last_uav_y);
    bresenham_param_t sample_line;
    
    double min_dist_to_obstacles_sqr = 
        pow(Params::g_uav_distance_to_obstacles / hmap->resolution(),2);
    double min_dist_to_other_uavs_sqr = 
        pow(Params::g_min_dist_to_other_uavs / hmap->resolution(),2);
    
    M_INFO2("Min dist to obstacles sqr %f ", min_dist_to_obstacles_sqr);
    M_INFO2("Min dist to other uavs sqr %f ", min_dist_to_other_uavs_sqr);
    M_INFO2("maxRangeStepsSqr %f ", maxRangeStepsSqr);
    
    get_bresenham_parameters(x_e, y_e, x2, y2, &sample_line);
    while ( get_next_point(&sample_line) ) {
        get_current_point(&sample_line, &x_uav, &y_uav);
        if (!verify(x_uav,y_uav,true))
           break;
        if ( (x_e == x_uav && y_e == y_uav)
          || (x2 == x_uav && y2 == y_uav)  ) {
            // ignores the first and last point on the line
            continue;
        }
        
        Pos uav_pose(x_uav,y_uav);
        
        if ( first_pos && Params::g_use_min_distances ) {
            if ( end_pos.sqrdist(uav_pose) < min_dist_to_obstacles_sqr ) 
            {
                // distance to the end_pos is too small
                // for the first uav of a line
                continue;
            }
        } else if ( Params::g_use_min_distances ){
            // same with last uav pos
            if ( last_uav_pos.sqrdist(uav_pose) < min_dist_to_other_uavs_sqr ) 
            {
                // distance to the end_pos is too small
                // for the first uav of a line
                continue;
            }
        }
        // same as above for x2,y2
        if ( end_pos2.sqrdist(uav_pose) < min_dist_to_obstacles_sqr 
            && Params::g_use_min_distances) 
        {
            // distance to the end_pos is too small
            // for the first uav of a line
            continue;
        }
        
        
        if ( end_pos.sqrdist(uav_pose) > maxRangeStepsSqr )
            break;
        
        double score = get_coverage_score( x_uav, y_uav, x_e, y_e, x2, y2,
            covered_poses);
        
        if ( DEBUG_LINE_TRAJECTORIES > 2) {
            M_INFO1("*** loc (%d,%d) score=%f \n",x_uav,y_uav,score);
        }
        if ( score > n_cells_max ) {
            n_cells_max = score;
            x_uav_max = x_uav; y_uav_max = y_uav;
        }
    }
    
    if ( DEBUG_LINE_TRAJECTORIES > 2) {
        M_INFO1("  Found best covered point location");
        M_INFO1(" (%d,%d) for line ",x_uav_max,y_uav_max);
        M_INFO1(" (%d,%d)--(%d,%d)\n",x_e,y_e,x2,y2);
    }
}


/*
 * adds the poses to VisSet covered_poses seen from x_uav,y_uav
 * and updates x_e, y_e to the new location
 *
 * Author: Andreas Kolling ( Fri Oct  5 18:27:36 CEST 2012 ) 
 */
void Visibility::get_coverage_poses(int x_uav, int y_uav, 
        int& x_e, int& y_e, 
        int x2, int y2, 
        VisSet& covered_poses ) {
    
    M_INFO2_D(DEBUG_LINE_TRAJECTORIES,2,"IMPRINTING (%d,%d) \n",x_uav,y_uav);
    bool broken1, broken2;
    int dummy_x = -1,dummy_y = -1;
    //from uav location to x_e,y_e limited to sensor range
    get_visible_line(x_uav, y_uav, x_e, y_e, dummy_x,dummy_y,
            covered_poses, covered_poses, broken1, false);
    x_e = dummy_x; y_e = dummy_y;
    //from uav location to x2,y2 limited to sensor range
    get_visible_line(x_uav, y_uav, x2, y2, dummy_x,dummy_y,
            covered_poses, covered_poses, broken2, true);
    if ( !broken1 ) {
        x_e = dummy_x; y_e = dummy_y;
        M_INFO2_D(DEBUG_LINE_TRAJECTORIES,2,
            "*** Not broken x_e,y_e %d %d \n",x_e,y_e);
    }
    M_INFO2_D(DEBUG_LINE_TRAJECTORIES,2, "*** NEW x_e,y_e %d %d \n",x_e,y_e);
}

/*
 * The coverage score counts the longest segment starting from x_e,y_e
 * that is contigously covered by either new points covered from 
 * x_uav,y_uav or in covered_poses
 *
 * Author: Andreas Kolling ( Fri Oct  5 14:10:06 CEST 2012 ) 
 */
double Visibility::get_coverage_score( int x_uav, int y_uav, int x_e, int y_e, int x2, int y2, VisSet& covered_poses ) {
    
    if ( DEBUG_LINE_TRAJECTORIES > 3 ) {
        M_INFO1(" get_coverage_score of location %d,%d \n",x_uav,y_uav);
    }
    
    // count how much of the current segment is covered contiguously
    VisSet covering_poses;
    bool broken = false;
    int x,y;
    if ( DEBUG_LINE_TRAJECTORIES > 3 ) {
        M_INFO1(" ---> look to x_e (%d,%d) \n",x_e,y_e);
    }
    double dist_covered = get_visible_line(x_uav, y_uav,x_e,y_e,x,y,
        covered_poses,covering_poses, broken, false);
    
    if ( !broken ) {
        if ( DEBUG_LINE_TRAJECTORIES > 3 ) {
            M_INFO1(" ---> look to x_2 (%d,%d) \n",x2,y2);
        }
        dist_covered += get_visible_line(x_uav, y_uav, x2,y2, x,y,
            covered_poses, covering_poses, broken, true);
    }
    return dist_covered;
}

/*
 * Computes the coverage from location x1,y1 towards x2,y2
 * Remembers the leftmost (closest to x2,y2) contiguous coverage segment
 * The forward flag is only used to compute the score if the
 * visible segement backwards was contiguous - when in doubt set to false
 * Author: Andreas Kolling ( Sun Oct  7 08:45:48 CEST 2012 ) 
 */

double Visibility::get_visible_line(int uav_x, int uav_y, 
        int x2, int y2,
        int& x_end, int& y_end, 
        VisSet& covered_poses, 
        VisSet& covering_poses, 
        bool& broken, bool forward) {
    // start at x,y, towards x2,y2, computes the next
    // longest chain of visibility
    covering_poses[Pos(uav_x,uav_y)] = true;
    broken = false;
    M_INFO3_D(DEBUG_LINE_TRAJECTORIES,4,
        "     get_visible_line from  %d,%d to %d,%d \n",uav_x,uav_y,x2,y2);
    double count = 0;
    bool have_new_end = false;
    bool lost_track = false;
    double h1 = getHeight(uav_x,uav_y) + pursuerHeight;
    double maxSlope = -HUGE_VAL;
    double wx1,wy1;
    double maxRangeSteps = get_max_steps();
    double maxRangeStepsSqr = pow(maxRangeSteps,2);
    hmap->grid2worldMM(wx1,wy1,uav_x,uav_y);
    Pos pos_uav(uav_x,uav_y);
    //long int steps = 0;
    double forward_count = -1;
    int last_x = -1, last_y = -1;
    bresenham_param_t line;
    int n_visible_traversables = 0;
    int n_visible_traversables_backwards = 0;
    get_bresenham_parameters(uav_x,uav_y, x2, y2, &line);
    while (get_next_point(&line)) {
        int x,y;  
        get_current_point(&line, &x, &y);
        Pos pos(x,y);
        
        if (!verify(x,y,true)) { break; }
        if ( pos.sqrdist( pos_uav ) > maxRangeStepsSqr ) { break;}
        
        M_INFO3_D(DEBUG_LINE_TRAJECTORIES,6,"          (%d,%d) ",x,y);
        
        bool traversable = hmap->is_cell_traversable(x,y);
        double wx,wy;
        hmap->grid2worldMM(wx,wy,x,y);
        double h2 = getHeight(x,y);
        double d = hypot(wx-wx1, wy-wy1);
        double m = (h2 - h1) / d;
        double m_t = ((h2 + targetHeight) - h1) / d;        
        
        if (m_t < maxSlope && traversable) { 
            // this point is a traversable hidden point
            VisSet::iterator it = covered_poses.find(pos);
            if ( it != covered_poses.end() && it->second == true ) {
                M_INFO3_D(DEBUG_LINE_TRAJECTORIES,6,"cov ");
            } else {
                M_INFO3_D(DEBUG_LINE_TRAJECTORIES,6,"unc ");
                if ( forward && forward_count == -1 ) {
                    // first traversable, hidden, and uncovered point
                    have_new_end = true;
                    x_end = x; y_end = y;
                    forward_count = n_visible_traversables;
                }
                lost_track = true;
                n_visible_traversables_backwards = 0;
                broken = true;
            }
        }
        else if (traversable) {
            // traversable and visible point
            if ( covered_poses.find(pos) == covered_poses.end() ) {
                // and also not yet covered
                n_visible_traversables++;
                n_visible_traversables_backwards++;
                if ( lost_track ) {
                    if ( !forward ) {
                        have_new_end = true;
                        x_end = x; y_end = y;
                    }
                    lost_track = false;
                }
                M_INFO3_D(DEBUG_LINE_TRAJECTORIES,6,"vis ");
                covering_poses[pos] = true;
            }
        }
        if (m >= maxSlope) { maxSlope = m; }
        last_x = x; last_y = y;
        M_INFO3_D(DEBUG_LINE_TRAJECTORIES,6,"\n");
    }
    count = n_visible_traversables_backwards;
    if ( !have_new_end ) {
        if ( forward ) {
            x_end = last_x; y_end = last_y;
            forward_count = n_visible_traversables;
        } else {
            x_end = uav_x; y_end = uav_y;
        }
    }
    if ( DEBUG_LINE_TRAJECTORIES > 5 ) {
        M_INFO3("x1(%d,%d) x2(%d,%d) new end %d,%d \n",uav_x,uav_y,x2,y2,x_end,y_end);
        M_INFO3("count=%f forw.=%f \n",count, forward_count);
        printf("Broken? %d\n",broken);
        printf("count=%f\n",count);
    }
    
    if ( forward )
        return forward_count;
    else
        return count;
}

int Visibility::computeVisibilitySet(int x, int y, VisSet& vset)
{
   // First check whether we find the set in the cache
   if (Params::g_use_visibility_cache && _visibilityCache.find(Pos(x,y)) != _visibilityCache.end()) {
      vset = _visibilityCache[Pos(x,y)];
   }
   else {
      vset.clear();
      vset[Pos(x,y)] = true;

      int count = 1;
      if (!Params::g_use_circular_fov) {
         // Simple hack: trace to all border cells as SQUARE
         for (int x2=0; x2<hmap->sizeX(); x2++) {
            count += computeVisibleLinePoints(x,y,x2,0,vset);
            count += computeVisibleLinePoints(x,y,x2,hmap->sizeZ()-1,vset);
         }

         for (int y2=0; y2<hmap->sizeZ(); y2++) {
            count += computeVisibleLinePoints(x,y,0,y2,vset);
            count += computeVisibleLinePoints(x,y,hmap->sizeX()-1,y2,vset);
         }
      }
      else if (Params::g_use_circular_fov) {
         int maxRangeSteps = pursuerRange / hmap->resolution();
         maxRangeSteps = std::min(maxRangeSteps, hmap->sizeX());
         maxRangeSteps = std::min(maxRangeSteps, hmap->sizeZ());
         deque<Pos> circ;
         computeCircle(maxRangeSteps, x, y, circ);
         for (unsigned int i=0; i<circ.size(); i++) 
            count += computeVisibleLinePoints(x,y,circ[i].x, circ[i].y - maxRangeSteps,vset);
         // Add extra circles to close all holes
         for (unsigned int i=0; i<circ.size(); i++) 
            count += computeVisibleLinePoints(x,y,circ[i].x+1, circ[i].y - maxRangeSteps,vset);
         for (unsigned int i=0; i<circ.size(); i++) 
            count += computeVisibleLinePoints(x,y,circ[i].x, circ[i].y+1 - maxRangeSteps,vset);
      }
      // Filter the set
      //if (filter)
      //   filterVisibilitySet(vset);


      //if (vset.size() < 2) {
      //   M_ERR("WARNING: Very small visibility set!\n");
      //}
      if ( Params::g_use_visibility_cache )
         _visibilityCache[Pos(x,y)] = vset;

   }
   return vset.size();
}


// Check if position is within map
bool Visibility::verify(int x, int y,bool silent) 
{
   if (hmap==NULL || x<0 || y<0 || x >= hmap->sizeX() || y >= hmap->sizeZ()) {
      if (!silent)
         M_ERR("Visibility: Pos %d %d out of map range!\n",x,y);
      return false;
   }
   return true;
}

// Get height value
double Visibility::getHeight(int x, int y) 
{
   if (!verify(x,y)) return -1.0;
   return hmap->getCellsMM()[x][y].getHeight() / 1000.0;
}

// Set Visibility
void Visibility::setVisible(int x, int y, bool v) 
{
   if (!verify(x,y))
      return;
   hmap->getCellsMM()[x][y].setVisible(v);
}

// Set Cleared
void Visibility::setCleared(int x, int y, bool v) 
{
   if (!verify(x,y))
      return;
   hmap->getCellsMM()[x][y].setCleared(v);
}


// Pos to index
int Visibility::posToIndex(const Pos& p)
{
   return p.y * hmap->sizeX() + p.x;
}

void Visibility::generateE() 
{
   for (int xx=0; xx<hmap->sizeX(); xx++) 
      for (int yy=0; yy<hmap->sizeZ(); yy++) 
         if ((hmap->getCellsMM()[xx][yy].getClass() == HeightCell::ELC_FLAT_GROUND)) 
		 {
            	E.insert(Pos(xx,yy));
		 }
}

Visibility::Pos Visibility::drawRandomFromE() 
{
   int pos = (int) (drand48() * (E.size()-1));
   set<Pos,Pos>::iterator it = E.begin();
   while (pos-- > 0)
      it++;
   Pos p = *it;
   E.erase(it);
   return p;
}


void Visibility::writeExperimentInfo(string filename)
{
   ofstream outfile;
   outfile.open(filename.c_str(),ios_base::out);
   outfile 
      << "# Meaning of each column in the experiment file"  << "\n"
      << "#1:mapname" << "\n " 
      << "#2:num_robots_shady" << "\n " 
      << "#3:num_robots_noshady" << "\n " 
      << "#4:num_ignored_edges_shady" << "\n " 
      << "#5:num_ignored_edges_noshady" << "\n " 
      << "#6:num_edges_shady" << "\n " 
      << "#7:num_edges" << "\n " 
      << "#8:num_vertices" << "\n " 
      << "#8:num_span_trees" << "\n " 
      << "#10:bias_spanning" << "\n " 
      << "#11:bias_spanning" << "\n "
      << "#12:pursuer_height" << "\n "
      << "#13:pursuer_range" << "\n "
      << "#14:target_height" << "\n "
      << "#15:use_improved_sampling" << "\n "
      << "#16:improved_sampling_min_area_size" << "\n "
      << "#17:use_only_one_region_per_pixel" << "\n "
      << "#18:min_size_for_polygon_at_vertex" << "\n "
      << "#19:generate_regular_instead_of_sparse_edges" << "\n "
      << "#20:compute_shady_edges" << "\n "
      << "#21:Time needed for computing graph" << "\n "
      << "#22:Time needed for computing strategy" << "\n "
      << endl;
   outfile.close();
}
 
vector<Visibility::Pos> Visibility::getAllBorderCells(int x, int y, int border)
{
   vector<Pos> border_region;

   deque<Pos> search;
   set<Pos,Pos> closed;
   search.push_back(Pos(x,y));
   closed.insert(Pos(x,y));

   while (!search.empty()) 
   {
      // Take out next element in queue
      Pos cP = search.front();
      search.pop_front();

      //printf("Search: ");
      //for (unsigned i=0; i<search.size(); i++)
      //   printf("(%d %d) ",search[i].x, search[i].y);
      //printf("\n");

      // Check if cell is of same border type
      vector<int> borders = hmap->getCellsMM()[cP.x][cP.y].getBorders();
      bool isContained = false;
      for (unsigned int j=0; j<borders.size(); j++) {
         if (borders[j] == border) {
            isContained = true;
            break;
         }
      }
      if (!isContained)
         continue;

      // Add pos to result
      border_region.push_back(cP);

      // Expand 
      for (int xx=-1; xx<=1; xx++) {
         for (int yy=-1; yy<=1; yy++) {
            if (verify(cP.x+xx,cP.y+yy,true) && closed.find(Pos(cP.x+xx,cP.y+yy)) == closed.end())  {
               search.push_back(Pos(cP.x+xx,cP.y+yy));
               closed.insert(Pos(cP.x+xx, cP.y+yy));
            }
         }
      }
   }
   return border_region;
}


bool Visibility::saveGraph(string filename)
{
   ofstream file (filename.c_str(), ios::out);
   if (!file.is_open()) {
      M_ERR("Error: could not open file: %s for writing\n",filename.c_str());
      return false;
   }

   // Header
   file << "digraph G {\n ";

   // Vertexes
   for (unsigned int i=0; i<vertices.size(); i++) 
   {
      file << vertices[i].id << " [pos = \"" 
         << (int) vertices[i].pos.x << ", " 
         << (int) vertices[i].pos.y
         << "\", shape=circle,fontsize=10,fontcolor=purple,color=blue];\n";
   }

   // Edges
   for (set<Edge>::iterator it=edges.begin(); it != edges.end(); it++) {
      if (it->shady)  
         file << it->id1 << " -> " << it->id2 << " [ label = \"shady\" ];\n";
      else	
         file << it->id1 << " -> " << it->id2 << " [ label = \"non-shady\" ];\n";
   }
   file << " }\n";

   if (Params::g_readWriteBorderAndRegionInfo) {
      file << "CONTAINS_BORDER_AND_REGION_INFO" << endl;
      M_INFO1("Writing also border and region info to file!\n");
   }
   else {
      file << "DOES_NOT_CONTAIN_BORDER_AND_REGION_INFO" << endl;
      M_INFO1("Skip writing of border and region info to file!\n");
   }

   // All additional height map stuff
   for (int xx=0; xx<hmap->sizeX(); xx++) 
   {
      for (int yy=0; yy<hmap->sizeZ(); yy++) 
      {
         // rgb
         double r,g,b;
         hmap->getCellsMM()[xx][yy].getRGB(r, g, b);
         file << r << " " << g << " " << b << endl;

         // distance
         double d = hmap->getCellsMM()[xx][yy].getDistance();
         file << d << endl;

         if (Params::g_readWriteBorderAndRegionInfo) {
            // regions
            vector<int> regions = hmap->getCellsMM()[xx][yy].getRegions();
            file << regions.size() << " ";
            for (unsigned j=0; j<regions.size(); j++)
               file << regions[j] << " ";
            file << endl;

            // borders
            vector<int> borders = hmap->getCellsMM()[xx][yy].getBorders();
            file << borders.size() << " ";
            for (unsigned j=0; j<borders.size(); j++)
               file << borders[j] << " ";
            file << endl;
         }
      }
   }

   file.close();
   return true;
}

bool Visibility::loadGraph(string filename)
{
   _filename = filename;
   // open file for reading
   ifstream file (filename.c_str(), ios::in);
   if (!file.is_open()) {
      M_ERR("Error: could not open file: %s for reading\n",filename.c_str());
      return false;
   }
   M_INFO2("Reading graph from file '%s'\n",filename.c_str());

   string line;

   // Check file format
   getline(file, line);
   if (line.find("digraph") == string::npos || line.size() < 1) {
      M_ERR("First line should contain 'digraph'. Couldn't find in line '%s'\n",line.c_str());
      return false;
   }

   vertices.clear();
   edges.clear();

   // Read the graph structure
   while (!file.eof()) {
      getline (file, line);
      if (line.size()<1)
         continue;

      // Parse Vertex
      if (line.find("pos") != string::npos && line.find("shape") != string::npos) {
         istringstream inStream(line);
         char buf[1024];
         char c;
         inStream.get(buf,1024,'[');
         int id = atoi(buf);
         inStream.get(buf,1024,'"');
         inStream.get(c); // read delim
         inStream.get(buf,1024,',');
         int x = atoi(buf);
         inStream.get(c); // read delim
         inStream.get(buf,1024,'"');
         int y = atoi(buf);
         if (DEBUG_LOAD)
            printf("VERTEX id:%d, xy = %d %d\n",id,x,y);
         vertices.push_back(Vertex(id, Pos(x,y)));
      }
      // Parse Edge
      else if (line.find("->") != string::npos) {
         istringstream inStream(line);
         char buf[1024];
         char c;
         inStream.get(buf,1024,'-');
         int id1 = atoi(buf);
         inStream.get(c);
         inStream.get(c); 
         inStream.get(buf,1024,'[');
         int id2 = atoi(buf);
         inStream.get(buf,1024,'"');
         inStream.get(c); 
         inStream.get(buf,1024,'"');
         bool shady = false;
         if (string(buf) == "shady")
            shady=true;
         else if (string(buf) == "non-shady")
            shady=false;
         else
            M_ERR("Error while parsing graph file!\n");


         if (DEBUG_LOAD)
            printf("EDGE %d ------> %d shady: %d\n",id1,id2,shady);
         edges.insert(Edge(id1, id2, shady));
      }
      else 
         break;
   }

   bool containsBorderAndRegionInfo = false;
   getline (file, line);
   if (line == "CONTAINS_BORDER_AND_REGION_INFO") {
      containsBorderAndRegionInfo = true;
      if (Params::g_readWriteBorderAndRegionInfo)
         M_INFO1("Reading also border and region info from file!\n");
      else
         M_WARN("Ignoring border and region info from file!\n");
   }
   else if (line == "DOES_NOT_CONTAIN_BORDER_AND_REGION_INFO") {
      if (Params::g_readWriteBorderAndRegionInfo)
         M_WARN("File does not contain border and region info!\n");
      else
         M_INFO1("No border and region info\n");
   }
   else {
      M_ERR("Wrong file format!\n");
      return false;
   }

   // Read all additional height map stuff
   for (int xx=0; xx<hmap->sizeX(); xx++) 
   {
      for (int yy=0; yy<hmap->sizeZ(); yy++) 
      {
         istringstream sline;
         // RGB Values
         if (file.eof()) {
            M_ERR("Error while reading from file: end reached\n");
            return false;
         }
         getline (file, line);
         sline.clear();
         sline.str(line);
         double r,g,b;
         sline >> r;
         sline >> g;
         sline >> b;
         hmap->getCellsMM()[xx][yy].setRGB(r, g, b);

         // Distance
         if (file.eof()) {
            M_ERR("Error while reading from file: end reached\n");
            return false;
         }
         getline (file, line);
         sline.clear();
         sline.str(line);
         double d;
         sline >> d;
         hmap->getCellsMM()[xx][yy].setDistance(d);

         if (containsBorderAndRegionInfo) {
            // Regions
            if (file.eof()) {
               M_ERR("Error while reading from file: end reached\n");
               return false;
            }
            hmap->getCellsMM()[xx][yy].getRegions().clear();
            getline (file, line);
            if (Params::g_readWriteBorderAndRegionInfo) {
               sline.clear();
               sline.str(line);
               int num=-1;
               sline >> num;
               while (num-- > 0) {
                  int rr;
                  sline >> rr;
                  hmap->getCellsMM()[xx][yy].getRegions().push_back(rr);
               }
            }

            // Borders
            if (file.eof()) {
               M_ERR("Error while reading from file: end reached\n");
               return false;
            }
            hmap->getCellsMM()[xx][yy].getBorders().clear();
            getline (file, line);
            if (Params::g_readWriteBorderAndRegionInfo) {
               sline.clear();
               sline.str(line);
               int num = -1;
               sline >> num;
               while (num-- > 0) {
                  int bb;
                  sline >> bb;
                  hmap->getCellsMM()[xx][yy].getBorders().push_back(bb);
               }
            }
         }
      }
   }

   file.close();
   return true;
}

void Visibility::detectRegionBorder(int x, int y, bool traversability)
{
   if (!verify(x,y)) 
      return;

   vector<Pos> vicinity;
   for (int xx=-1; xx<=1; xx++) {
      for (int yy=-1; yy<=1; yy++) {
         if (verify(x+xx,y+yy,true)) {
            vicinity.push_back(Pos(x+xx,y+yy));
         }
      }
   }

   // Check whether any obstacle around
   bool hasObstacleBorder = false;
   if (traversability) {
      if ( Params::g_use_only_one_region_per_pixel )
      {
         if (hmap->getCellsMM()[x][y].getClass() != HeightCell::ELC_FLAT_GROUND ||
               (Params::g_use_shrubs &&  hmap->getCellsMM()[x][y].getVegetation() > 0))
            hasObstacleBorder = true;
         
      }
      else {
         for (unsigned int i=0; i<vicinity.size(); i++) {
            Pos v = vicinity[i];
            if (hmap->getCellsMM()[v.x][v.y].getClass() != HeightCell::ELC_FLAT_GROUND ||
                  (Params::g_use_shrubs &&  hmap->getCellsMM()[v.x][v.y].getVegetation() > 0))
               hasObstacleBorder = true;
         }
      }
   }

   // Check for each region point if it is also border point
   vector<int> regionSet1 = hmap->getCellsMM()[x][y].getRegions();
   for (vector<int>::iterator it=regionSet1.begin(); it!=regionSet1.end(); it++) 
   {
      int region1 = *it;
      bool isRegionBorder = false;

      for (unsigned int i=0; i<vicinity.size(); i++) {
         Pos v = vicinity[i];
         // Check whether region is contained in every neighbor
         bool contained = false;
         vector<int> regionSet2 = hmap->getCellsMM()[v.x][v.y].getRegions();
         for (vector<int>::iterator it2=regionSet2.begin(); it2!=regionSet2.end(); it2++) {
            if (region1 == *it2) {
               contained = true;
               break;
            }
         }
         if (!contained) {
            isRegionBorder = true;
            break;
         }
      }
      if (traversability && hasObstacleBorder)
         isRegionBorder = false;
      if (isRegionBorder) {
         hmap->getCellsMM()[x][y].getBorders().push_back(region1);
         if (hmap->getCellsMM()[x][y].getRegions().size() > 2)
            hmap->getCellsMM()[x][y].setIsCoveredBorder(true);
      }
   }
}


std::ostream & operator << (std::ostream& o, const Visibility::Pos& p)
{
   o << "(" << p.x << "," << p.y << ")" << " ";
   return o;
}

std::ostream & operator << (std::ostream& o, const Visibility::Vertex& v)
{
   o << "id:" << v.id << " " << v.pos << " ";
   return o;
}

std::ostream & operator << (std::ostream& o, const Visibility::Edge& e)
{
   o << e.id1 << " ---> " << e.id2 << " ";
   return o;
}


void Visibility::tesCircle(int radius, int x0, int y0)
{
   deque<Pos> arc;
   computeCircle(radius, x0, y0, arc);

   for (unsigned int i=0; i<arc.size(); i++)
      setVisible(arc[i].x,arc[i].y,true);
}

void Visibility::computeCircle(int radius, int x0, int y0, deque<Pos> &arc)
{
   int f = 1 - radius;
   int ddF_x = 0;
   int ddF_y = -2 * radius;
   int x = 0;
   int y = radius;

   arc.clear();
   arc.push_back(Pos(x0, y0));

   vector<Pos> tarc1;
   vector<Pos> tarc2;
   while(x < y)
   {
      if(f >= 0)
      {
         y--;
         ddF_y += 2;
         f += ddF_y;
      }
      x++;
      ddF_x += 2;
      f += ddF_x + 1;

      if (left) {
         tarc1.push_back(Pos(x0 + x, y0 - y + radius));
         tarc2.push_back(Pos(x0 + y, y0 - x + radius));

         tarc1.push_back(Pos(x0 - x, y0 - y + radius));
         tarc2.push_back(Pos(x0 - y, y0 - x + radius));

         tarc1.push_back(Pos(x0 + x, y0 + y + radius));
         tarc2.push_back(Pos(x0 + y, y0 + x + radius));

         tarc1.push_back(Pos(x0 - x, y0 + y + radius));
         tarc2.push_back(Pos(x0 - y, y0 + x + radius));
      }
      else {
         tarc1.push_back(Pos(x0 + x, y0 + y - radius));
         tarc2.push_back(Pos(x0 + y, y0 + x - radius));

         tarc1.push_back(Pos(x0 - x, y0 + y - radius));
         tarc2.push_back(Pos(x0 - y, y0 + x - radius));

         tarc1.push_back(Pos(x0 + x, y0 - y - radius));
         tarc2.push_back(Pos(x0 + y, y0 - x - radius));

         tarc1.push_back(Pos(x0 - x, y0 - y - radius));
         tarc2.push_back(Pos(x0 - y, y0 - x - radius));
      }
   }
   for(unsigned i = 0; i < tarc1.size(); i++)
      arc.push_back(tarc1[i]);
   for(unsigned i = 0; i < tarc2.size(); i++)
      arc.push_back(tarc2[tarc2.size() - 1 - i]);
}

void Visibility::computeDistanceMap()
{
   M_INFO3("Computing dmap ... ");
   double distanceThreshold = 8000.0;
   //double distanceThreshold = Params::g_obstacleMaxDist * 1000.0;
   // Set initial values
   for (int xx=0; xx<hmap->sizeX(); xx++) {
      for (int yy=0; yy<hmap->sizeZ(); yy++) { 
         double h = getHeight(xx, yy); 
         hmap->getCellsMM()[xx][yy].setDistance(h);
      }
   }
   double ds = sqrt(2.0) * hmap->resolution();
   double ls = hmap->resolution();

   // compute distances
   for (int x=2; x<hmap->sizeX()-1; x++) 
      for (int z=2; z<hmap->sizeZ()-1; z++)
      {
         float mval = distanceThreshold;
         mval=mval<hmap->getCellsMM()[x-1][z-1].getDistance() + ds ? mval:  hmap->getCellsMM()[x-1][z-1].getDistance() + ds;
         mval=mval<hmap->getCellsMM()[x-1][z].getDistance()   + ls ? mval:  hmap->getCellsMM()[x-1][z].getDistance()   + ls;
         mval=mval<hmap->getCellsMM()[x-1][z+1].getDistance() + ds ? mval:  hmap->getCellsMM()[x-1][z+1].getDistance() + ds;
         mval=mval<hmap->getCellsMM()[x][z-1].getDistance()   + ls ? mval:  hmap->getCellsMM()[x][z-1].getDistance()   + ls;
         mval=mval<hmap->getCellsMM()[x][z+1].getDistance()   + ls ? mval:  hmap->getCellsMM()[x][z+1].getDistance()   + ls;
         mval=mval<hmap->getCellsMM()[x+1][z-1].getDistance() + ds ? mval:  hmap->getCellsMM()[x+1][z-1].getDistance() + ds;
         mval=mval<hmap->getCellsMM()[x+1][z].getDistance()   + ls ? mval:  hmap->getCellsMM()[x+1][z].getDistance()   + ls;
         mval=mval<hmap->getCellsMM()[x+1][z+1].getDistance() + ds ? mval:  hmap->getCellsMM()[x+1][z+1].getDistance() + ds;
         mval=mval<distanceThreshold?mval:distanceThreshold;
         double d = hmap->getCellsMM()[x][z].getDistance() < mval ? hmap->getCellsMM()[x][z].getDistance() : mval;
         hmap->getCellsMM()[x][z].setDistance(d);
      }

   for (int x=hmap->sizeX()-2; x>1; x--)
      for (int z=hmap->sizeZ()-2; z>1; z--)
      {
         float mval=distanceThreshold;
         mval=mval<hmap->getCellsMM()[x-1][z-1].getDistance() + ds ? mval:  hmap->getCellsMM()[x-1][z-1].getDistance() + ds;
         mval=mval<hmap->getCellsMM()[x-1][z].getDistance()   + ls ? mval:  hmap->getCellsMM()[x-1][z].getDistance()   + ls;
         mval=mval<hmap->getCellsMM()[x-1][z+1].getDistance() + ds ? mval:  hmap->getCellsMM()[x-1][z+1].getDistance() + ds;
         mval=mval<hmap->getCellsMM()[x][z-1].getDistance()   + ls ? mval:  hmap->getCellsMM()[x][z-1].getDistance()   + ls;
         mval=mval<hmap->getCellsMM()[x][z+1].getDistance()   + ls ? mval:  hmap->getCellsMM()[x][z+1].getDistance()   + ls;
         mval=mval<hmap->getCellsMM()[x+1][z-1].getDistance() + ds ? mval:  hmap->getCellsMM()[x+1][z-1].getDistance() + ds;
         mval=mval<hmap->getCellsMM()[x+1][z].getDistance()   + ls ? mval:  hmap->getCellsMM()[x+1][z].getDistance()   + ls;
         mval=mval<hmap->getCellsMM()[x+1][z+1].getDistance() + ds ? mval:  hmap->getCellsMM()[x+1][z+1].getDistance() + ds;
         mval=mval<distanceThreshold?mval:distanceThreshold;
         double d = hmap->getCellsMM()[x][z].getDistance()<mval ? hmap->getCellsMM()[x][z].getDistance() : mval;
         hmap->getCellsMM()[x][z].setDistance(d);
      }

   M_INFO3(" ... done.\n");
}

void Visibility::writeDistancePPM(string filename)
{
   printf("Save Colored Distance PPM to %s\n",filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P6" << endl;
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

   int count = 1;
   for(int z = 0; z < hmap->sizeZ(); z++) {
      for(int x = 0; x < hmap->sizeX(); x++) {
         double d = hmap->getCellsMM()[x][z].getDistance();
         if (index.find(d) == index.end()) {
            c[count][0] = drand48() * 255;
            c[count][1] = drand48() * 255;
            c[count][2] = drand48() * 255;
            index[d] = count;
            count++;
            if (count>=MAX_COL) 
               count = 0;
         }
      }
   }

   for(int z = 0; z < hmap->sizeZ(); z++) {
      for(int x = 0; x < hmap->sizeX(); x++) {
         double d = hmap->getCellsMM()[x][z].getDistance();
         int idx = index[d];
         char r = (unsigned char) c[idx][0];
         char g = (unsigned char) c[idx][1];
         char b = (unsigned char) c[idx][2];
         //cout << r << " " << g << " " << b << endl;
         os.put(r);
         os.put(g);
         os.put(b);
      }
   }
}


void Visibility::writeDistancePGM(string filename)
{
   printf("Save Distance PGM to %s\n",filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P5" << endl;
   os << hmap->sizeX() << " " << hmap->sizeZ() << " " << 255 << endl;

   // Find max
   double max = 0.0;
   double min = HUGE_VAL;
   for(int z = 0; z < hmap->sizeZ(); z++) {
      for(int x = 0; x < hmap->sizeX(); x++) {
         double d = hmap->getCellsMM()[x][z].getDistance();
         if (d>max)
            max=d;
         if (d<min)
            min=d;
      }
   }

   printf("MAX DIST IS %lf\n",max);
   printf("MIN DIST IS %lf\n",min);
   for(int z = 0; z < hmap->sizeZ(); z++) {
      for(int x = 0; x < hmap->sizeX(); x++) {
         double f = 255.0 * (hmap->getCellsMM()[x][z].getDistance() / max);
         char c = (unsigned char) f;
         os.put(c);
      }
   }
}

void Visibility::writeClassesPPM(string filename)
{
   printf("Save Colored Classes PPM to %s\n",filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P6" << endl;
   os << hmap->sizeX() << " " << hmap->sizeZ() << endl;
   os << 255 << endl;

   for(int z = 0; z < hmap->sizeZ(); z++) {
      for(int x = 0; x < hmap->sizeX(); x++) {
         int cl = hmap->getCellsMM()[x][z].getClass();
         char r,g,b;
         if (cl != HeightCell::ELC_FLAT_GROUND) {
            r = (unsigned char) 255;
            g = (unsigned char) 0;
            b = (unsigned char) 0;
         }
         else  {
            r = (unsigned char) 0;
            g = (unsigned char) 0;
            b = (unsigned char) 0;
         }
         os.put(r);
         os.put(g);
         os.put(b);
      }
   }
}


void Visibility::testingWriteAllBorderComponentsToPPM(char* filename)
{
   // TESTING
   M_INFO2("Extracting Border Components ...");
   fflush(stdout);
   int bcount = 0;
   for (int xx=0; xx<hmap->sizeX(); xx++) {
      for (int yy=0; yy<hmap->sizeZ(); yy++) {
         if (hmap->getCellsMM()[xx][yy].isBorderCell()) 
         {
            vector<int> borders = hmap->getCellsMM()[xx][yy].getBorders();

            for (unsigned j=0; j<borders.size(); j++) {
               vector<Pos> b = getAllBorderCells(xx, yy, borders[j]);

               // Assign color to line
               for (unsigned int i=0; i<b.size(); i++) 
                  hmap->getCellsMM()[b[i].x][b[i].y].setDistance(bcount);
               bcount++;
            }
         }
      }
   }
   M_INFO2("...done\n");
   writeDistancePPM(filename);
}


void Visibility::markNodeVisSetAsClear(int nodeId, bool erase)
{
   VisSet vset;
   if (computeVisibilitySet(nodeId, vset) >= 0) {
      // Unmark all
      if (erase) 
         eraseMarkings(false, true);

      printf("Marking set of %d cells as cleared!\n",(int) vset.size());
      for (VisSet::iterator it=vset.begin(); it != vset.end(); it++)  {
         Pos p = it->first;
         setCleared(p.x,p.y,true);
      }
   }
}

int Visibility::filterOutAssignedCells(VisSet& vset)
{
   int removed = 0;
   // remove all points that have an associated region already
   for (Visibility::VisSet::iterator it=vset.begin(); it != vset.end(); )  
   {
      Visibility::Pos p = it->first;
      it++;
      vector<int> regions
         = hmap->getCellsMM()[p.x][p.y].getRegions();
      if ( regions.size() >= 1 ) { // already associated
         vset.erase(p);
         removed++;
      }
      
   }
   return removed;
}


bool Visibility::computeRegionSet(string base_filename, int num_span_trees, bool bias_spanning)
{
   int shady_counter = 0, full_edge_counter = 0;
   bool do_compute_graph = (Params::g_always_recompute_graph || edges.empty() || vertices.empty());
   // For time measurments   
   double startT = 0;double endGraphT=0; double endStrategyT=0;
   startT= getCurrentTime();

   if (do_compute_graph) 
   {
      // Clear all old stuff
      edges.clear();
      vertices.clear();
      for (int xx=0; xx<hmap->sizeX(); xx++) {
         for (int yy=0; yy<hmap->sizeZ(); yy++) { 
            hmap->getCellsMM()[xx][yy].setDistance(0.0);
            hmap->getCellsMM()[xx][yy].getRegions().clear();
            hmap->getCellsMM()[xx][yy].getBorders().clear();
            hmap->getCellsMM()[xx][yy].setRGB(0.5,0.5,0.5);
         }
      }

      int count = 0;

      // Generate vertexes
      if (Params::g_use_improved_sampling)
      {
		printf("Using improved sampling!\n");
         HmAbstraction* hmabs = new HmAbstraction(hmap);
         deque<Visibility::Pos > result;
         hmabs->generateMaxDetectionSet(getPursuerHeight(), getPursuerRange(), getTargetHeight() ,result);
         delete hmabs;
         for (deque<Visibility::Pos>:: iterator it = result.begin(); it != result.end(); it++) 
         {
            Pos p = *it;
            
            // Generate visibility set
            VisSet vset;
            computeVisibilitySet(p.x, p.y, vset);
            int removedCells = 0;
            if (Params::g_use_only_one_region_per_pixel)
            {
               if (!Params::g_generate_regular_instead_of_sparse_edges)
                  removedCells = filterOutAssignedCells( vset );
               filterVisibilitySet(vset, true, p.x,p.y);
            }
            double r = drand48();
            double g = drand48();
            double b = drand48();
            //cout << p << " " << count << " VSET SIZE " << vset.size() << endl;
            if (DEBUG_WRITE_PPMS) {
               char name1[100];
               sprintf(name1,"/tmp/FINALvSet_V%d_.bmp",(int) result.size()); 
               if ( vset.size() >= 5 )
                  writeVisSetToFile( string(name1), vset );
            }
            
            if ( (vset.size() + removedCells) >= Params::g_imporved_sampling_min_area_size )
            {
               for (VisSet::iterator it=vset.begin(); it != vset.end(); it++)  {
                  Pos p_vis = it->first;
                  // color cells according to region color
                  //if ( hmap->getCellsMM()[p_vis.x][p_vis.y].getRegions().size() > 0 )
                  //   cout << " ERROR FILTER DID NOT WORK!" << endl;
                  hmap->getCellsMM()[p_vis.x][p_vis.y].getRegions().push_back(count);
                  hmap->getCellsMM()[p_vis.x][p_vis.y].setRGB(r,g,b);
               }
               vertices.push_back(Vertex(count, p));
               count++;
            }
         }
      }
      else {
		printf("Using random sampling!\n");
	
         // Generate E
         generateE();

		printf("E has the size %d\n",(int) E.size());

         while (!E.empty()) {
            // Draw random cell
            Pos p = drawRandomFromE();

            // Generate visibility set
            VisSet vset;
            computeVisibilitySet(p.x, p.y, vset);

            int removedCells = 0;
            if (Params::g_use_only_one_region_per_pixel)
            {
               if (!Params::g_generate_regular_instead_of_sparse_edges)
                  removedCells = filterOutAssignedCells( vset );
               filterVisibilitySet(vset, true, p.x,p.y);
            }
 
            double r = drand48();
            double g = drand48();
            double b = drand48();

            bool added = false;
            if ( (vset.size() + removedCells) >= Params::g_imporved_sampling_min_area_size ) {
               vertices.push_back(Vertex(count, p));
               //printf("Add --> ");
               added=true;
            }

            // Remove vis set from E
            //printf("Removing %d\n",(int)vset.size());
            for (VisSet::iterator it=vset.begin(); it != vset.end(); )  {
               Pos p_vis = it->first;
               it++;
               if (added) {
                  // color cells according to region color
                  hmap->getCellsMM()[p_vis.x][p_vis.y].setRGB(r,g,b);
                  hmap->getCellsMM()[p_vis.x][p_vis.y].getRegions().push_back(count);
               }
               E.erase(p_vis);
            }
            //printf("Size after remove %d\n",(int)E.size());

            if (added)
               count++;
         }
      }

      printf("Created %d vertexes\n",count);

      // Compute border cells and mark them white
      for (int xx=0; xx<hmap->sizeX(); xx++) 
         for (int yy=0; yy<hmap->sizeZ(); yy++) {
            detectRegionBorder(xx, yy, true);
            if (hmap->getCellsMM()[xx][yy].isBorderCell()) {
               //hmap->getCellsMM()[xx][yy].setRGB(1.0,1.0,1.0);
            }
         }
      //M_INFO2("Computed border cells\n");

      vector<Pos> dummy1;
      vector< vector <Pos> > dummy2( vertices.size()  , dummy1);
      vector< vector < vector <Pos> > > borders_region_intersect( vertices.size(), dummy2);
      // borders_region_intersect[j][jj] === \delta D(v_j) \intersect D(v_{jj}) 

      // fill borders_region_intersect
      for (int xx=0; xx<hmap->sizeX(); xx++) 
      {
         for (int yy=0; yy<hmap->sizeZ(); yy++) 
         {
            vector<int> borders = hmap->getCellsMM()[xx][yy].getBorders();
            for (unsigned j=0; j<borders.size(); j++) 
            {
               // xx,yy is a border pixel for D(v_j)
               // check vicinity (new, previously we only checked xx,yy)
               vector<Pos> vicinity;
               for (int xxx=-1; xxx<=1; xxx++) {
                  for (int yyy=-1; yyy<=1; yyy++) {
                     if (verify(xx+xxx,yy+yyy,true)) {
                        vicinity.push_back(Pos(xx+xxx,yy+yyy));
                     }
                  }
               }
               for (unsigned int i=0; i<vicinity.size(); i++) 
               {
                  Pos v = vicinity[i];
                  vector<int> regions = hmap->getCellsMM()[v.x][v.y].getRegions();
                  for ( unsigned jj=0; jj<regions.size(); jj++ )
                  {
                     // D(v_jj) also sees this pixel xx,yy
                     if ( borders[j] != regions[jj] )
                        borders_region_intersect[ borders[j] ][ regions[jj] ].push_back( Pos(xx,yy) );
                  }
               }               
            }
         }
      }

      vector< int > dummy3( vertices.size() );
      vector< vector<int> > edge_matrix( vertices.size(), dummy3 );
      for ( unsigned j=0; j<vertices.size(); j++) 
      {  
         for ( unsigned jj=0; jj<vertices.size(); jj++ )
         {
            edge_matrix[j][jj] = 0;
         }
      }

      unsigned counter = 0;

      for ( unsigned j=0; j<vertices.size(); j++) 
      {
         for ( unsigned jj=0; jj<vertices.size(); jj++ )
         {
            if ( jj == j )
               continue;
            if ( borders_region_intersect[j][jj].size() == 0 )
               continue;
            // check whether the intersection of \delta D(v_j) and D(v_{jj})
            // is covered entirely by another region
            vector<int> covering_regions;
            bool fill_first = true;
            // go through all points of the intersection of \delta D(v_j) and D(v_{jj})
            for ( unsigned i=0; i < borders_region_intersect[j][jj].size(); ++i )
            {
               // going through all points of the intersection of 
               // \delta D(v_j) and D(v_{jj})
               if ( !fill_first && covering_regions.size() == 0)
                  break;
               Pos pose = borders_region_intersect[j][jj][i];
               vector<int> regions = hmap->getCellsMM()[pose.x][pose.y].getRegions();
               if ( fill_first )
               {
                  covering_regions = regions; 
                  fill_first = false;
               }
               // go through all covering regions and see whether coverage is broken
               vector<int> new_covering_regions;
               for ( unsigned ii=0; ii < covering_regions.size(); ++ii )
               {
                  // is covering_regions[ii] in regions?
                  if ( std::find( regions.begin(), regions.end(), covering_regions[ii] ) != regions.end() )
                  {
                     if ( covering_regions[ii] != int(j) && covering_regions[ii] != int(jj) )
                        new_covering_regions.push_back( covering_regions[ii] );
                  }               
               }
               covering_regions = new_covering_regions;
            }

            if ( covering_regions.size() > 0 )
            {
               edge_matrix[j][jj] = 2;
            }
            else
            {
               edge_matrix[j][jj] = 1;
            }
            ++counter;
            for ( unsigned i=0; i < borders_region_intersect[j][jj].size(); ++i )
            {
               Pos pose = borders_region_intersect[j][jj][i];
               hmap->getCellsMM()[pose.x][pose.y].setDistance( counter );
            }
         }
      }

      // Clean up matrix and make symmetric
      for ( unsigned j=0; j<vertices.size(); j++) 
      {
         for ( unsigned jj=0; jj<vertices.size(); jj++ )
         {
            edge_matrix[j][jj] = std::min( edge_matrix[j][jj], edge_matrix[jj][j] );
         }
      }

      // Graph construction
      // Vertexes
   }
  

   endGraphT = getCurrentTime();

   // Write statistics from experiment
   if (Params::g_run_experiment_mode) {
      static bool first = true;
      if (first) {
         writeExperimentInfo("EXPERIMENT_STATS.info");
         first=false;
      }
      ofstream outfile;
      outfile.open("EXPERIMENT_STATS.txt",ios_base::app);
      outfile 
         << base_filename.c_str() << " "
         << shady_counter << " "
         << full_edge_counter << " "
         << num_span_trees << " "
         << bias_spanning << " "
         << Params::g_pursuer_height << " "
         << Params::g_pursuer_range << " "
         << Params::g_target_height << " "
         << Params::g_use_improved_sampling << " "
         << Params::g_imporved_sampling_min_area_size << " "
         << Params::g_use_only_one_region_per_pixel << " "
         << Params::g_min_size_for_polygon_at_vertex << " "
         << Params::g_generate_regular_instead_of_sparse_edges << " "
         << Params::g_compute_shady_edges << " "
         << endGraphT-startT << " "
         << endStrategyT - endGraphT 
         << endl;
      outfile.close();
   }
return do_compute_graph;
}

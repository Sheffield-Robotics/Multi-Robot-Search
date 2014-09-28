/*
 * A collection of functions to polygoniaze various map structures using the 
 *
 * Author: Andreas Kolling ( Tue Aug 28 10:37:45 CEST 2012 ) 
 */
#ifndef POLYGONIZATION_H
#define POLYGONIZATION_H

#include "polygonization/alpha_shape.h"
#include "polygonization/polygon_environment.h"
#include "heightmap/heightmap.h"

#include <utility>
#include <list>

namespace polygonization {

/*
 * Polygonialize a heightmap using a as alpha for the alpha shape and e as 
 * epsilon for the line simplification.
 *
 * Author: Andreas Kolling ( Tue Aug 28 10:51:51 CEST 2012 ) 
 */
 
Polygon_Environment* 
polygonize_heightmap( HeightMap* map, float a, float e, 
    int start_x, int start_y, std::string filename );

Polygon_Environment* 
polygonize_img( bool** occ, int max_x, int max_y, float a, float e, 
        int start_x, int start_y, std::string filename );

// should we switch this to a class that remembers 
class Polygonization {
  public:
    Polygon_Environment* _e;
    
    std::list< std::pair<int,int> > _to_be_visited;
    
    Polygon_Environment*
    polygonize_occ_vector( bool** occ, int max_x,int max_y,
        float a, float e, int start_x, int start_y, std::string pure_filename );
    
    Polygon_Environment* 
    polygonize_heightmap( HeightMap* map, float a, float e, 
        int start_x, int start_y, std::string pure_filename );
        
    
        
    bool** o;
    bool** v;
  private:
    
    int size_x;
    int size_y;
  
    void 
    save_occ_map(string pure_filename);
  
    void 
    blow_up_obstacles();
      
    void 
    fill_out_inaccessible_areas(int start_x, int start_y);
    
    void 
    visit_neighbors(int,int);
    
};

}
#endif
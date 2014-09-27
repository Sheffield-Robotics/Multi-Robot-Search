// 
//  polygonialization.cpp
//  icra12_pe_code
//  
//  Created by Andreas on 2012-08-28.
//  Copyright 2012 Andreas Kolling. All rights reserved.
// 

#include "polygonization/polygonization.h"

namespace polygonization {

    using namespace std;

    Polygon_Environment* 
    polygonize_heightmap( HeightMap* map, float a, float e, 
            int start_x, int start_y, std::string filename ) {
        Polygonization polygonization_inst;
        return polygonization_inst.polygonize_heightmap( map, a, e, start_x,start_y,
            filename );
    }


    /*
     * This function creates a lot of dynamic memory via 'new'
     * bool** occ and E and A will have to be deleted properly
     *
     * Author: Andreas Kolling ( Fri Sep  7 16:23:04 CEST 2012 ) 
     */
    polygonization::Polygon_Environment* 
    Polygonization::polygonize_heightmap( HeightMap* map, float a, float e, 
            int start_x, int start_y, std::string pure_filename ) {
    
        size_x = map->sizeX();
        size_y = map->sizeZ();
    
        o = new bool*[size_x];
        // get a bool occupancy vector from map
        for (int x=0; x < size_x; x++) {
            o[x] = new bool[size_y];
            for (int y=0; y < size_y; y++) {
                if ( x == 0 || y == 0 || x == size_x-1 || y == size_y-1
                 || x == 1 || y == 1 || x == size_x-2 || y == size_y-2) {
                    // boundary is always occupied
                    o[x][y] = true;
                } 
                else if ( map->is_cell_traversable(x,y)) {
                    o[x][y] = false;
                } 
                else {
                    o[x][y] = true;
                }
            }
        }
    
        // save the occupancy map
        string filename = pure_filename + ".ppm";
        ofstream os;
        os.open(filename.c_str());
        os << "P5" << endl << size_y << " " << size_x << " " << 255 << endl;
        for(int y=size_y-1; y>=0; y--) {
            for(int x=0; x<size_x; x++) {
                double f = 255.0;
                if ( o[x][y] == true )
                    f = 0.0;
                char c = (unsigned char) f;
                os.put(c);
            }
        }
        os.close();
    
        // blow up each obstacle by a bit
        if ( Params::g_blow_up_obstacles == 1 ) {
            bool** oo = new bool*[size_x];
            for (int x=0; x < size_x; x++) {
                oo[x] = new bool[size_y];
                for (int y=0; y < size_y; y++) {
                    oo[x][y] = false;
                }
            }
            for (int x=0; x < size_x; x++) {
                for (int y=0; y < size_y; y++) {
                    if ( o[x][y] == true ) {
                        oo[x][y] = true;
                        for ( int i = -1; i < 2; i++ ) {
                            for ( int j = -1; j < 2; j++ ) {
                                int nx = x+i; int ny = y+j;
                                if ( nx < 0 || nx >= size_x 
                                  || ny < 0 || ny >= size_y
                                  || (j==0 && i==0)) {
                                    continue;
                                }
                                oo[nx][ny] = true;
                            }
                        }
                    }
                }
            }

            for (int x=0; x < size_x; x++) {
                delete o[x];
            }
            delete o;
            o = oo;
        }
    
    
        // fill out the inaccessible parts
        v = new bool*[size_x];
        for (int x=0; x < size_x; x++) {
            v[x] = new bool[size_y];
            for (int y=0; y < size_y; y++) {
                v[x][y] = false;
            }
        }
        visit_neighbors(start_x,start_y);
        std::cout << " done visiting " << std::endl;
        //go through visited and mark all those not visited as occupied
        for (int x=0; x < size_x; x++) {
            for (int y=0; y < size_y; y++) {
                if ( !v[x][y] ) {
                    o[x][y] = true;
                }
            }
        }
    
    
    
        Alpha_shape* A = new Alpha_shape;
        A->construct( o, map->sizeX(), map->sizeZ(), a );
        Polygon_Environment* E = new Polygon_Environment;
        E->construct( A, e );
        return E;
    }

    void Polygonization::visit_neighbors( int start_x, int start_y) {
        _to_be_visited.push_back( pair<int,int>(start_x,start_y) );
        v[start_x][start_y] = true;
        int x,y;
        while( _to_be_visited.size() > 0 ) {
            //std::cout << " _to_be_visited.size() " 
            //    << _to_be_visited.size()<<std::endl;
            x = _to_be_visited.front().first;
            y = _to_be_visited.front().second;
            _to_be_visited.pop_front();
            for ( int i = -1; i < 2; i++ ) {
                for ( int j = -1; j < 2; j++ ) {
                    int nx = x+i; int ny = y+j;
                    if ( nx < 0 || nx >= size_x 
                      || ny < 0 || ny >= size_y
                      || (j==0 && i==0)) {
                        continue;
                    }

                    if ( !(v[nx][ny]) && !(o[nx][ny])) {
                        v[nx][ny] = true;
                        _to_be_visited.push_back( pair<int,int>(nx,ny) );
                    }
                }
            }
        }
    
        return;
    }

}
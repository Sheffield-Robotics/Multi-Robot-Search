#ifndef MY_ALPHA_SHAPE_H
#define MY_ALPHA_SHAPE_H

#include "geometry_types.h"

#include <iostream>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Triangulation_default_data_structure_2.h>
#include <CGAL/spatial_sort.h>
#include <CGAL/Triangulation_hierarchy_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>

namespace polygonization {

    typedef CGAL::Alpha_shape_vertex_base_2<KERNEL> Alpha_vertex;
    typedef CGAL::Triangulation_vertex_base_with_info_2<bool,KERNEL,Alpha_vertex> 
        Triangulation_Vertex;
    typedef CGAL::Triangulation_hierarchy_vertex_base_2<Triangulation_Vertex>  
        Triangulation_hierarchy_vertex;
    typedef CGAL::Triangulation_face_base_2<KERNEL> Triangulation_face;
    typedef CGAL::Alpha_shape_face_base_2<KERNEL,Triangulation_face> Alpha_face;

    typedef CGAL::Triangulation_default_data_structure_2<KERNEL,
                Triangulation_hierarchy_vertex,Alpha_face> 
        Triangulation_data_structure;
    typedef CGAL::Delaunay_triangulation_2<KERNEL,Triangulation_data_structure>
        Delaunay_triangulation;
    typedef CGAL::Triangulation_hierarchy_2<Delaunay_triangulation> 
        Triangulation_hierarchy;
    typedef CGAL::Alpha_shape_2<Triangulation_hierarchy> Alpha_shape_base;

    class Alpha_shape : public Alpha_shape_base {
      private:
        void set_neighbors_occupied(const int x, const int y);
        int _size_x;
        int _size_y;
      public:
        typedef std::vector<Point> Point_vector;
        typedef Alpha_shape_base::Alpha_shape_vertices_iterator vertex_iterator;
        typedef Alpha_shape_base::Vertex_handle vertex_handle;
        typedef Alpha_shape_base::Edge_circulator edge_circulator;
        typedef Alpha_shape_base::Alpha_shape_edges_iterator edge_iterator;
    
        bool** _occupied;
        Point_vector _occ_points;    
        Alpha_shape() : _size_x(0), _size_y(0), _occupied(NULL) {};
        ~Alpha_shape() { _occ_points.clear(); };
        
        void construct( bool** occ, int size_x, int size_y, float a );
        
        vertex_handle get_neighbor_vertex( edge_circulator e, vertex_handle v );
        bool is_regular( edge_circulator e );
        bool is_regular( edge_iterator e );
        int regular_edges( vertex_handle v );
        void savePPMobstacles(const char* filename );
        
        bool isInside(int x, int y) const {
            return x >= 0 && y >= 0 && x < _size_x && y < _size_y;
        };
        
        bool isOccupied(int x, int y) const { 
            if (isInside(x,y)) {return _occupied[x][y];} return false;
        };
    };

}

#endif
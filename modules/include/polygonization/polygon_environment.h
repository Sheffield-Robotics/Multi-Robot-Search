#ifndef Polygon_Environment_H
#define Polygon_Environment_H

#include "utilities/misc.h"
#include "utilities/angles.h"
#include "polygonization/geometry_util.h"
#include "polygonization/polygon.h"
#include "polygonization/alpha_shape.h"
#include "polygonization/voronoi_diagram.h"
#include "polygonization/dp_line_simplification.h"
#include "polygonization/visilibity.hpp"
#include "polygonization/segment_visibility_graph.h"
#include <CGAL/Polygon_2_algorithms.h>
#include <vector>
#include <list>
#include <limits>  

#define DEBUG_POLYGON_ENVIRONMENT 6

namespace polygonization {

    typedef std::vector<Polygon> Polygons;

    class Voronoi_Diagram;
    
    class Polygon_Environment : public Polygons {
      public:    
        Alpha_shape* A;
        Voronoi_Diagram* VD;
        // the primary polygon (simply-connected)
        Polygon* master_polygon;
        Vis_graph* visibility_graph;
        VisiLibity::Environment* my_environment;
        Segment_Visibility_Graph* seg_vis_graph;
        double** distance_mat;
        double** distance_mat2;
        Segment*** segment_mat;
        int n_polygons;
        int open_polygons;
        double _epsilon;
        double _visi_epsilon;
        
        std::vector<VisiLibity::Visibility_Polygon> visi_polies;
        std::vector<Segment_Visibility_Graph::vertex>   
            seg_vis_graph_type1_vertices;
        std::vector<Segment_Visibility_Graph::vertex>
            seg_vis_graph_type2_vertices;
        
        std::vector<Point> artificial_points;
        int is_artificial2( Point p );
        bool is_artificial( Point p );
        bool is_artificial( Segment s );
        bool is_artificial( int i );
        bool is_same_articifial( int i, int j );
        
        
        Polygon_Environment() {n_polygons = 0;};
        void simplify( double epsilon );
        void simplify_polygon( int poly, double epsilon );
        void construct( Alpha_shape* alphaShape, float epsilon );
        void add_to_master(Polygon* poly, int poly_id, int goal);

        void extract_polygon( Alpha_shape::vertex_handle v );
        
        void get_visibility_polygon(Segment s);
        void process_master_polygon();
        void compute_visibility_polygon_at_vertex(VisiLibity::Point &p);
        VisiLibity::Point get_visi_vertex(int i);
        bool index_bound_check( int i );
        int 
        get_segment_index_for_point( VisiLibity::Point p );
        
        int
        is_endpoint( KERNEL::Point_2 p );
        bool
        vertex_is_reflexive( int i, 
            VisiLibity::Visibility_Polygon& v_poly );
        int get_next_index( int i );
        int get_prev_index( int i );
        
        KERNEL::Point_2 get_point_of_segment_on_segment( KERNEL::Segment_2 s, int k, bool& success ); 
        
        
        void update_seg_to_seg(int i, int j,KERNEL::Segment_2 s, double d);
        
        void set_edges_type_1_to_infty(int i,int j);
        void reset_edges_type_1_from_infty();
        Segment_Visibility_Graph::vertex special_v;
        
        std::vector<Segment_Visibility_Graph::edge_descriptor> extra_edges;
        void remove_extra_edges();
        void add_extra_edges_for(int seg_ind);
        void add_extra_edges(int i, int j);
        Segment_Visibility_Graph::edge_descriptor
        add_edge_to_visibility_graph( int i,  int type_i, int j, int type_j, double d, double x = std::numeric_limits<double>::max(), double y = std::numeric_limits<double>::max(), double x2=std::numeric_limits<double>::max(),double y2=std::numeric_limits<double>::max());
        
        bool is_necessary_block(int i, int j);        
        bool is_necessary_split(int i, int j, int k);
        
        
        std::list<KERNEL::Segment_2>
        get_shortest_path(int i, int j, double& final_dist);
        std::vector< std::vector < std::list<KERNEL::Segment_2> > >     
            *get_shortest_path_cache;
        std::vector< std::vector < double > >     
            *get_shortest_path_distance_cache;
        std::vector< std::vector < bool > >     
            *get_shortest_path_cache_filled;
        bool check_path_cache(int i, int j);
        std::list<KERNEL::Segment_2> get_path_cache(int i, int j);
        double get_path_distance(int i, int j);
        void set_path_cache(int i, int j, std::list<KERNEL::Segment_2> l, double d);
        
        
        std::list<KERNEL::Segment_2>
        plan_in_svg(Segment_Visibility_Graph::vertex v,
            Segment_Visibility_Graph::vertex w,  double& total_path_distance);
            
        void remove_special_vertex_direct_visible();
            
        void remove_special_vertex_edges();
        
        double get_block_distance(int i, int j);
        int get_block_cost(int i, int j, double r = 1);
        bool get_split_distances(int i, int j, int k, 
            double& d1, double& d2, double r = 1);
        int get_split_cost(int i, int j, int k, double r = 1);
        std::list<KERNEL::Segment_2>
        shortest_split_cost(int i, int j, int k, double& cost);
        std::list<KERNEL::Segment_2>
        shortest_split_costs(int i, int j, int k, 
            double& cost1, double& cost2, int& split_point_index);
        
        void fix_index(int &i) {
            if ( master_polygon->size() <= 0 ) { return; }
            while ( i < 1 ) {
                i = i + int(master_polygon->size());
            }
            while ( i > int(master_polygon->size()) ) {
                i = i - master_polygon->size();
            }
        }
            
        
        
        double
        shortest_distance_between( KERNEL::Segment_2 s, KERNEL::Point_2 p,
             KERNEL::Point_2& closest_point );
         
        Segment_Visibility_Graph::vertex
        get_segment_visibility_vertex(int i, int type_i);
        
        KERNEL::Point_2 
        Point_2_from_poly_vertex( VisiLibity::Point& p );        
        
        void compute_to_master_distances(int goal);
        double get_distance(int i, int j);
        void set_distance(int i, int j, double d);
        double get_distance2(int i, int j);
        void set_distance2(int i, int j, double d);
        Segment get_segment(int i, int j, int k);
        void set_segment(int i, int j, int k, Segment s);
        void swap_segment( int i, int g, int min_i );
        bool is_in_polygon( Segment s, Polygon* poly, Polygon::Vertex_iterator& it, int& source_to_target );
        bool get_connections_segments( Segment& shortest_s, Segment& master_s, Segment&poly_s, Polygon::Vertex_iterator& poly_i, Polygon::Vertex_iterator& master_i, int poly_id, int goal, Polygon* poly);
        
        double shortest_path_distance_between(double x1,double y1,double x2, double y2);
        
        void process_visibility_polygon(
            int main_segment_index,
            KERNEL::Point_2 v, 
            VisiLibity::Visibility_Polygon &v_poly, bool special_run = false);
        
        
        /*
         * Turns an environment consisting of multiple polygons into 
         * an environment with only one polygon by connecting polygons
         * that are close to each other 
         * Author: Andreas Kolling ( Tue Aug 28 13:27:23 CEST 2012 ) 
         */
        void make_simply_connected();
        
      private:
        void visit_alpha_vertex( Alpha_shape::vertex_handle v_start );
        bool find_next_regular_vertex( Alpha_shape::vertex_handle v, 
            Alpha_shape::vertex_handle& vr );
        bool find_next_regular_vertex( Alpha_shape::vertex_handle v, 
            Alpha_shape::vertex_handle v_prev, 
            Alpha_shape::vertex_handle& vr ); 
        bool find_last( Alpha_shape::vertex_handle v, 
            Alpha_shape::vertex_handle v_start );
            
        std::vector< std::vector<bool> >*  seg_to_seg_visible;
        std::vector< std::vector<double> >*  seg_to_seg_distance;
        std::vector< std::vector<KERNEL::Segment_2> >*  seg_to_seg_segments;

    };
    bool pair_comparison (std::pair<int,double> a, std::pair<int,double> b);
    bool pair_comparison_inv (std::pair<int,double> a, std::pair<int,double> b);
}

#endif
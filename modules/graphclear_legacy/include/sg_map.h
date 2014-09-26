#ifndef SG_MAP
#define SG_MAP

//Headers (fold)
//******* Basic Headers *******
#include <iostream>
#include <fstream>              
#include <math.h>              
#include <unistd.h>
#include <stdio.h>
//******* Config Headers *******
#include <libconfig.h++>
using namespace libconfig;
//******* OpenCV Headers *******
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
//******* GSL Headers *******
#include <gslwrap/matrix_float.h>
//******* STL Headers *******
#include <queue>
#include <deque>
#include <set>
#include <vector>
#include <utility>
//******* Boost Graph Library Headers *******
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>
//******* Custom Headers *******
#include "define.h"
#include "sg_graph.h"
#include "sg_map_point.h"
#include "br_pose.h"
#include "wave_point.h"
#include "CGAL_types.h"

typedef double HOMOG[3];
#define XX 0
#define YY 1
#define WW 2
#define CROSSPROD_2CCH(p, q, r) /* 2-d cartesian to homog cross product */\
 (r)[WW] = (p).x() * (q).y() - (p).y() * (q).x();\
 (r)[XX] = - (q).y() + (p).y();\
 (r)[YY] =   (q).x() - (p).x();
#define DOTPROD_2CH(p, q)	/* 2-d cartesian to homog dot product */\
 (q)[WW] + (p).x()*(q)[XX] + (p).y()*(q)[YY]

// (end)

#define EPSILON 0.05

using namespace gsl;
using namespace std;
using namespace boost;

typedef std::pair< VD_Vertex, sg_vertex_d > vd_sg_vertex;
typedef std::vector< vd_sg_vertex > vd_sg_vertex_vector;
//******* MAP POINTS: list(1d),lists(2d) & grid (3d) + queue *******
typedef std::pair< int , int >
  map_point;
typedef std::vector< map_point >
  map_point_list;
typedef std::vector< map_point_list >
  map_point_lists;
typedef std::vector< map_point_lists >
  map_point_grid;
typedef std::queue< map_point >
	map_point_queue;

//******* SG_MAP POINTS: advanced map points *******  
typedef std::vector< sg_map_point >
  pointvector;
typedef std::vector< pointvector >
	pointvector_list;
typedef std::vector< CGAL_Point >
  CGAL_pointvector;
typedef std::vector< pair< float, pointvector::iterator> >
  distancevector;

//******* Wave Points *******
typedef std::vector< wave_point >
  wave_point_vector;  
typedef std::priority_queue<wave_point, std::deque<wave_point>,ltstr> 
	wavepointqueue;

//******* sg_vertex + sg_vertex pair: list(1d) + grid(2d) *******
typedef pair<sg_vertex_d, sg_vertex_d> 
	sg_vertex_d_pair;
typedef std::vector< sg_vertex_d_pair > 
  vertex_pair_list;
typedef std::vector<std::vector< sg_vertex_d > > 
  vertexgrid;  

////struct K : CGAL::Exact_predicates_inexact_constructions_kernel {};
//typedef CGAL::Alpha_shape_cell_base_3<K>            Fb;
//typedef CGAL::Triangulation_data_structure_3<Fb> Tds;
//typedef CGAL::Delaunay_triangulation_3<K,Tds>       Triangulation_3;
//typedef CGAL::Alpha_shape_3<Triangulation_3>        Alpha_shape_3;
//typedef Alpha_shape_3::Alpha_iterator       Alpha_iterator;
//typedef Alpha_shape_3::Vertex_handle        Vertex_handle;
//typedef std::list<Vertex_handle>::const_iterator Vl_it;

class sg_map {
  private:
	int    partial_goal_point_x;
	int    partial_goal_point_y;
    //******* Visualization Options *******
	int    show_maps; // if 1: displays maps
	int    save_imgs; // if 1: saves the images when displayed
	//******* Map parameters *******
	
	int    n_occupied_cells;
	//******* Parameters for graph extraction *******
	int    minima_travel_dist;
	int    minima_distance;
	int    minima_inc_corridor;
	int    minima_non_dec_corridor;
	int    minima_non_dec_travel_dist;
	float  minima_increase_lvl;
	float  minima_margin;
	int    sense_range_pix;
	int    sense_delta;
	double minimum_gap;
	double simplify_epsilon;
	double alpha_value;
	int    n_polygons;
	//******* Datastructures to describe graph & map *******
	
	//******* Boundary Description *******
	// Occupancy grid - also used for imprinting edge lines
	matrix_float occ;
	matrix_float boundary;
	// A vector of all occupied points
	pointvector occ_points;
	pointvector boundary_points; 
	CGAL_pointvector CGAL_occ_vector;
	Polygons poly_boundary;
	Polygons poly_boundary_simple;

	std::set< CGAL_VD::Halfedge > visited_halfedges;
	std::set< VD_Vertex > visited_vertices;
	vd_sg_vertex_vector vd_sg_pairs;

	//******* Wave Algorithm Datastructures *******
	// 2d matrix describes how many waves collided on this point
	matrix_float wave_count;
	// Clearance at this point - distance to closest obstacle
	matrix_float clearance_value;
	// For each point closest_obstacle contains the closest obstacles
	map_point_grid* closest_obstacle;
	// Describes to which polygon segment this point belongs
	matrix_float to_polygon;
	// A vector of all voronoi points
	pointvector voronoi_points;
	// mark which VD edges/vertices are visited
	matrix_float edge_visited;
	matrix_float vertex_visited;
	matrix_float voronoi_matrix;
	// Voronoi Segments/Edges 
	matrix_float fortune_v_segment; 
	// Corresponding segment ids for boundary points
	matrix_float belongs_to_segment;
	// Sparse matrix with voronoi minima inscribed
	matrix_float voronoi_minima;
	// A vector of voronoi_minima - short 
	pointvector voronoi_minima_points;
	// Points that will not be considered for a new minimum
	matrix_float blocked_for_minimum;
	//for each point assign a vertex
	vertexgrid* vertex_membership;
	// a list of points for each vertex, i.e. those belonging to it
	map_point_lists points_for_vertex;
	// points on a voronoi edge
	map_point_lists points_on_edge;
	map_point_lists points_on_vertex;
	map_point_list  vertex_points;
	// The complete Voronoi Diagram
	CGAL_VD voronoi_diagram;
	// The complete surveillance graph
	sg_graph G_g;
	// A temporary variable to remember a list of segments to traverse for a sweep
	std::vector<int> temp_segment_vector;
	matrix_float path_points;    
	//******* Helper datastructures to aid processing ******* (fold)
	// Stores vertices that will have to be merged
	// 1-dim vector of sg_vertex_d
	vertex_pair_list ready_for_merge;
	// direction of the wave that hit the point
	// helper to process wave collisions
	matrix_float wavepoints_hitdir;
	// when visiting voronoin points this describes
	// whether the point has been processed in visit_voronoi_point
	matrix_float visited;     
	matrix_float visited_close_min;
	// when processing the waves we go through this priority queue
	wavepointqueue wavepoints;
	//*** (end)
	//******* Helpers ******* (fold)
	float dis_qui(int a, int b);
	float distance(int a, int b, int c, int d);
	void get_eight_case(int case_nr, int& ii, int& jj);
	void get_sixteen_case(int case_nr, int& ii, int& jj);
	bool bound_check(int n_x, int n_y);
	void add_vertex_membership(int i, int j, sg_vertex_d vd);
	void add_closest_obstacle(int i, int j, int cause_x, int cause_y);
	void reset_closest_obstacles(int i, int j);
	void set_occ(int row, int col, int value);
	int get_occ(int row, int col);
	void add_occ_pt(int i, int j);
	sg_map_point get_occ_pt(int i);
    //*** (end)
	//******* Private Display / Print Functions ******* (fold)
	void 
		write2img_posevector( IplImage* img, br_pose_vector poses);
	void 
		write2img_posevector_real( IplImage* img, br_pose_vector poses);
	void 
		write2img_clearance_value(IplImage* img);
	void 
		write2img_voronoi_diagram(IplImage* img);
	void 
		write2img_voronoi_diagram_segments(IplImage* img);
	void 
		write2img_voronoi_diagram_segments_alive(IplImage* img);
	void 
		write2img_voronoi_minima(IplImage* img);
	void
		write2img_segment_association( IplImage* img);
	void 
		write2img_graph(IplImage* img);
	void 
		write2img_causes_for_minima(IplImage*);
	void 
		write2img_sg_vertex_crits( IplImage* img, int from_dir );
	void 
		write2img_sg_vertices(IplImage* img, bool display_text, bool display_boxes);
	void 
		write2img_sg_edges(IplImage* img, bool display_text);   
	void
		write2img_occupancy( IplImage* img);
	void
		write2img_boundary( IplImage* img);
	void 
		write2img_visited(IplImage* img);
	void 
		write2img_polygon( IplImage* img, Polygon& poly );
	void 
		write2img_CGAL_voronoi_vertices( IplImage* img );
	//*** (end)

    //******* Boundary Simplification *******
	void 
		simplify_polygon( int poly_id );
    //******* Voronoi Diagram Computation ******* (fold)
	void 
		compute_fortune_voronoi_diagram();
	//*** (end)
	void 	
		alpha_shape();
	void 
		build_CGAL_voronoi_diagram();
	void 
		imprint_CGAL_voronoi_diagram();
	void 
		process_CGAL_voronoi_diagram();
	void 
		visit_VD_edge( CGAL_VD::Edge_iterator edge_it );
	sg_vertex_d
		visit_VD_vertex( VD_Vertex vert );
	bool 
		find_sg_vertex( double x, double y, sg_vertex_d& vd );
	bool 
		find_sg_vertex( VD_Vertex v , sg_vertex_d& vd ) ;
	
	void 
		construct_weights_SG_graph();
	void 
		critical_points( SDG::Edge e, sg_vertex_d vd, int n_edges );
	void 
		critical_edge_points( SDG::Edge e, sg_edge_d ed );
	int 
		number_of_robots( double d );
	void 
		save_as_img_critical_points( SDG::Edge e );
	void 
		set_vertex_visited( VD_Vertex v );
	void 
		set_edge_visited( CGAL_VD::Halfedge e );
	bool 
		is_vertex_visited( VD_Vertex v );
	bool 
		is_edge_visited( CGAL_VD::Halfedge e );
	bool 
		is_edge_valid( CGAL_VD::Halfedge e );
	double 
	  distance_to_site( VD_Site_2 s, SDG::Point_2 pt );
	double 
		distance_site_to_site( VD_Segment_2, VD_Segment_2, 
		                       SDG::Point_2&, SDG::Point_2&);
	double 
	  distance_to_sites( VD_Site_2 s1, VD_Site_2 s2, SDG::Point_2 pt );
	double 
	  distance_to_site( VD_Site_2 s, SDG::Point_2 pt, SDG::Point_2& c_pt  );
	double 
	  point_to_segment_distance( double nSx1, double nSy1, double nSx2,
	                             double nSy2, double nPx, double nPy,
	                             double *nCercaX,  double *nCercaY);
	double 
	  point_to_point_distance(double lOldX, double lOldY,
	                          double lNewX, double lNewY);
	void 
	  show_voronoi_diagram();
	void 
	  find_split( int i, int j, int *split, double *dist, Polygon& P);
	void 
	  dp_simplify( Polygon& P, std::vector<int>& final_indices );
		//******* Segmenting the Voronoi Diagram ******* (fold)
    int num_voronoi_segments;
    std::vector<int> dead_end_segments;
    void 
      f_v_visit_point( int i, int j, int current_segment );
    void 
      f_v_visit_point_mod( int i, int j, int current_segment );
    void 
      f_v_parse_vertex_points();
    void
      f_v_check_vertex_cluster( int i, int j );
    void 
      f_v_parse_segments();
    void 
      f_v_associate_segments();
    int 
      f_v_number_non_segmented_neighbors( int i, int j);
    bool
      is_dead_end( int segment_id );
		//*** (end)
		
	//******* Voronoi Minima Pipeline Processing ******* (fold)
	void 
	  compute_voronoi_minima_p();
	void 
	  voronoi_minimum_check( sg_map_point point );
    void 
      add_voronoi_minimum( int i, int j);		
		bool 
		  is_increasing_direction(int i, int j, float best_minima, int dist_travelled);
		bool 
      is_not_decreasing(int i, int j, float best_minima, int dist_travelled);
	  void 
      block_close_points_for_minima( int i, int j, int dist_travelled );
    //*** (end)
		
		//******* Bare Graph Creation ******* (fold)
    
    void 
      process_minima_to_vertices();
    void 
      process_minima_to_vertices_visit( int i, int j, sg_vertex_d vd );
    sg_vertex_d 
      add_new_vertex(int i, int j);
    void 
      process_minima_to_edges();
		sg_edge_d 
		  add_new_edge(sg_vertex_d vd1, sg_vertex_d vd2);
		//*** (end)
		
		//******* Edge lines ******* (fold)
		
		void
      compute_edge_lines();
		void 
      compute_edge_line( sg_edge_d ed );
    void 
      get_edge_line( int& x_1, int& x_2, int& y_1, int& y_2, sg_edge_d edge);
    void 
      imprint_closest_obstacle();
		void 
		  imprint_closest_obstacle(int x0, int y0, int x1, int y1 );

		//*** (end)
		
	  //******* Process Vertex membership & positions**** (fold)
		
		void 
		  process_vertex_membership();
		void 
		  process_vertex_membership_vertex(sg_vertex_d vd);
		void 
		  find_points_for_vertex( sg_vertex_d vd, int i, int j, bool starting);
		void
      compute_graph_vertex_position(sg_vertex_d vd);
		void 
		  compute_graph_vertex_positions();
		
    //*** (end)

    //******* Weight Computation ******* (fold)

		void 
		  compute_graph_vertex_weights();
		void 
		  compute_graph_vertex_weight(sg_vertex_d vert_desc);
		int 
		  bounding_box_cost( int ll_x, int ll_y, int ur_x, int ur_y );
    void 
      compute_graph_edge_weights();
    void 
      compute_graph_edge_weight( sg_edge_d ed );
      
		//*** (end)

		//******* Graph Improvement ******* (fold)
		void merge_vertices(sg_vertex_d vd1, sg_vertex_d vd2);
		void kill_vertex( sg_vertex_d vd );
		int merged_vertex_weight( sg_vertex_d vd1, sg_vertex_d vd2 );
		bool easy_merge_leaf( sg_vertex_d leaf);
		bool check_for_easy_leaf_merge( sg_vertex_d vd1, sg_vertex_d vd2, sg_edge_d ed);
		bool easy_merge_two( sg_vertex_d vd_merged);
		bool check_for_easy_two_merge( sg_vertex_d vd1, sg_vertex_d vd2, sg_edge_d ed, sg_edge_d ed2);
		void merge( sg_vertex_d leaf, sg_vertex_d vd2 );
		int edge_cost( sg_vertex_d vd );
		//*** (end)
						
  public:
	CvSize img_map_size;
	const char* base_img_dir;
	int nrows;
	int ncols;
	float resolution;
	// Voronoi Diagram created by the fortune algorithm
	// 0 == no voronoi point
	// 1 == voronoi point
	matrix_float fortune_voronoi;
	Config* robot_cfg;
	int leaf_mergers;
	int two_mergers;    
    
	//******* Constructor and Constructions *******
	sg_map( Config* the_config );
	void set_sense_range_pix( int range);
	//******* The entire graph construction process ******* (fold)
	void 
	  process_img(const char* img_filename, double lower_thres);
	void 
	  process_map();
	sg_graph* 
	  get_graph();
	//*** (end)
	//******* Public Display & Print Functions ******* (fold)
	void
		write2img_edge_crit_pts( IplImage* img,  sg_edge_d ed);
	void 
		write2img_vertex_crit_pts( IplImage*, sg_vertex_d, sg_vertex_d);
	void 
		write2img_polygon_boundary_simple( IplImage* img );
	void 
		write2img_sg_vertex_crits( IplImage* img, sg_vertex_d vd, int from_dir );
	void 
		save_img_posevector( const char* filename, br_pose_vector poses );
	void 
		save_img_posevector_real( const char* filename, br_pose_vector poses );
	void 
		save_img_clearance_value( const char* filename );
	void 
		display_graph(bool display_text, bool display_boxes, char* filename, char* filename2);
	void
		save_img_occupancy( const char* filename );
	void
		save_img_boundary( const char* filename );
	void 
		save_img_voronoi_segments( const char* filename );
	void
		save_img_segment_association( const char* filename );
	void 
		save_img_voronoi_segments_alive( const char* filename );
	void
		save_img_voronoi_diagram( const char* filename );
	void
		save_img_voronoi_minima( const char* filename );
	void 
		save_img_diagram_cost_minima( const char* filename );
	void 
		save_img_polygon( const char* filename, Polygon& poly );
	void 
		save_img_poly_boundary_simple( const char* filename );
	void 
		save_img_CGAL_voronoi_diagram( const char* filename );
	void 
		save_img_sg_vertex_crits( const char* filename, int from_dir);
	void 
		save_img_sg_vertex_all_crits( const char* filename );
	void 
		display_visited( const char* filename );
	void 
		cout_wavepoint_cost();
	void 
		cout_cause(int i, int j);
	void 
		cout_all_causes();
	void 
		cout_merger();
	//*** (end) 
	bool 
		easy_merge_leaves();
	bool 
		easy_merge_twos();
	int 
		count_vertices();
	void 
		cout_graph_summary();
	void 
		cout_graph_diagnostic( int lid);
	void 
		cout_alive_vertices();
	void 
		cout_vertex( sg_vertex_d vd );
	void 
		save_img_graph(const char* filename);
	int 
		get_voronoi_neighbors( int neighbors[][2], int i, int j);
	sg_vertex_d 
		get_vertex_membership(int i, int j);
	br_pose_matrix*
		generate_pose_matrix( sg_vertex_d vd, sg_edge_d ed );
	void
		add_poses( VD_Point_2 goal_p, VD_Point_2 star_p, int n_poses, br_pose_matrix* pose_mat );
	void
		write2img_pose_matrix( IplImage* img, br_pose_matrix* pose_mat );
	br_pose_vector 
		follow_main_voronoi_segments( sg_vertex_d vertex );
	int 
		follow_segment( int segment_id,  int current_x, int current_y );
	br_pose_vector 
		follow_main_voronoi_segment_path( sg_vertex_d vertex, sg_edge_d edge1, sg_edge_d edge2 );
	int 
		follow_segment_path( int segment_id,  int current_x, int current_y, int goal_x, int goal_y );
	br_pose_vector 
		follow_main_voronoi_segment_path2( sg_vertex_d vertex, sg_edge_d edge1, int& reached_x, int& reached_y );
	int 
		follow_segment_path2( int segment_id,  int current_x, int current_y );
	void 
		erase_from_segment_vector( int first_index, int last_index);
	void 
		segment_vector_status();      
};

class custom_ostream {
 public:
	custom_ostream( IplImage* img ) { stream_img = img; };
	~custom_ostream( ) { };
	custom_ostream& operator= (const custom_ostream& o) 
		{ stream_img = o.stream_img; return *this;};
	IplImage* stream_img;
};

template < class R >
custom_ostream&
operator<<(custom_ostream &gv, const CGAL::Point_2<R> &p)
{
	//cout << " called Point_2 << operator " << endl;
	return gv;
}

template < class R >
custom_ostream&
operator<<(custom_ostream &gv, const CGAL::Segment_2<R> &s)
{
	
	//cout << " called << segment operator" << endl;
	IplImage* img = gv.stream_img;
	int x1,x2,y1,y2;
	x1 = int(s.source().x()); y1 = int(s.source().y());
	x2 = int(s.target().x()); y2 = int(s.target().y());
	CvScalar s1 = cvGet2D( img, x1, y1 );
	CvScalar s2 = cvGet2D( img, x2, y2 );
	if ( s1.val[0] == 0 || s2.val[0] == 0 ) {
		// black is inside obstacle
	}
	else {
		// draw something close to black
		cvLine( img,
		        cvPoint( y1, x1 ),
		        cvPoint( y2, x2 ),
		        cvScalar( 1, 1, 1 ), 1);
	}
	return gv;
}

template < class R >
custom_ostream&
operator<<(custom_ostream &gv, const CGAL::Line_2<R> &r)
{
	//cout << " called line << operator " << endl;
	return gv;
}

template < class R >
custom_ostream&
operator<<(custom_ostream &gv, const CGAL::Ray_2<R> &r)
{
	//cout << " called ray << operator " << endl;
 	return gv;
}

#endif

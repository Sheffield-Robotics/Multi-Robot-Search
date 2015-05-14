#ifndef MY_CGAL_CUSTOM_TYPES
#define MY_CGAL_CUSTOM_TYPES

// IMPORTANT: this fixes undefined constants for squared_distance_2
//#include <CGAL/Kernel/global_functions_internal_2.h>
#include <CGAL/basic.h>
//#include <CGAL/squared_distance_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>

#include <CGAL/intersections.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Bbox_2.h>

#include <CGAL/Homogeneous.h>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_2.h>

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Partition_2/Vertex_visibility_graph_2.h>

#include <list>

namespace polygonization {

// typedef CGAL::Exact_predicates_inexact_constructions_kernel KERNEL;
// typedef CGAL::Cartesian<float> KERNEL;
// typedef CGAL::Cartesian<CGAL::Gmpq> Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel KERNEL;

typedef KERNEL::Point_2 Point;
typedef KERNEL::Vector_2 Vector;
typedef KERNEL::Direction_2 Direction;
typedef KERNEL::Segment_2 Segment;

typedef CGAL::Polytope_distance_d_traits_2<KERNEL> Poly_Traits;
typedef CGAL::Polytope_distance_d<Poly_Traits> Polytope_distance;

// typedef CGAL::Arr_segment_traits_2<KERNEL>              Arr_Traits;
typedef CGAL::Vertex_visibility_graph_2<KERNEL> Vis_graph;

// typedef CGAL::Polygon_with_holes_2<K> Polygon_wh;
// typedef CGAL::Polytope_distance_d_traits_2<K> Traits;
//// RT and NT template values default to K:RT
////typedef CGAL::Polytope_distance_d_traits_2<K, double, double> Traits;
// typedef CGAL::Polytope_distance_d<Traits> Polytope_distance;
// typedef CGAL::Creator_uniform_2<int, Point>               Creator;
// typedef CGAL::Random_points_in_square_2<Point, Creator>   Point_generator;
typedef CGAL::Creator_uniform_2<int, Point> Creator;
typedef CGAL::Random_points_in_square_2<Point, Creator> Point_generator;

// typedef CGAL::Circle_2<K>    Circle;
// typedef CGAL::Line_2<K>      Line;
// typedef CGAL::Ray_2<K>       Ray;
// typedef CGAL::Triangle_2<K>  Triangle;
// typedef CGAL::Iso_rectangle_2<K> Cgal_Rectangle;
// typedef CGAL::Bbox_2           BBox;
// typedef CGAL::Aff_transformation_2<K>  Transformation;
}

#endif
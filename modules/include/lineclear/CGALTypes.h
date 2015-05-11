#ifndef CGALTYPES_H
#define CGALTYPES_H

#include <list>

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
#include <CGAL/Bbox_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>

#include <CGAL/Aff_transformation_2.h>

// Qt headers
#include <QtGui>
#include <QString>
#include <QFileDialog>
#include <QGraphicsLineItem>
// GraphicsView items and event filters (input classes)
#include <CGAL/Qt/GraphicsViewPolylineInput.h>
#include <CGAL/Qt/PolygonGraphicsItem.h>
#include <CGAL/Qt/PolygonWithHolesGraphicsItem.h>
#include <CGAL/Qt/LineGraphicsItem.h>

namespace lineclear {

//typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point;
typedef K::Vector_2 Vector;
typedef K::Direction_2 Direction;
typedef K::Segment_2 Segment;

typedef CGAL::Polygon_2<K> Polygon;
typedef CGAL::Polygon_set_2<K> Polygon_set;
typedef CGAL::Polygon_with_holes_2<K> Polygon_wh;

typedef CGAL::Polytope_distance_d_traits_2<K> Traits;
// RT and NT template values default to K:RT
//typedef CGAL::Polytope_distance_d_traits_2<K, double, double> Traits;
typedef CGAL::Polytope_distance_d<Traits> Polytope_distance;

typedef CGAL::Creator_uniform_2<int, Point>               Creator;
typedef CGAL::Random_points_in_square_2<Point, Creator>   Point_generator;

typedef CGAL::Circle_2<K>    Circle;
typedef CGAL::Line_2<K>      Line;
typedef CGAL::Ray_2<K>       Ray;
typedef CGAL::Triangle_2<K>  Triangle;
typedef CGAL::Iso_rectangle_2<K> Cgal_Rectangle;
typedef CGAL::Bbox_2           BBox;


typedef CGAL::Arr_segment_traits_2<K>              Arr_Traits;
typedef Arr_Traits::Curve_2                        Arr_Segment;
typedef Arr_Traits::Curve_2                        Arr_Segment;
typedef CGAL::Arrangement_2<Arr_Traits>            Arrangement;

typedef CGAL::Aff_transformation_2<K>  Transformation;



}

#endif
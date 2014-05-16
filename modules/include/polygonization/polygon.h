#ifndef MY_POLYGON_H
#define MY_POLYGON_H

#include "geometry_types.h"

namespace polygonization {
    
    typedef CGAL::Polygon_2<KERNEL> Polygon_base;
    
    class Polygon : public Polygon_base {
      public:
        Polygon() : cached_circumference(0.0),cached_area(0.0) {}
        double cached_circumference;
        double cached_area;
        
        double distance_to( Polygon* other );
        void print();
        Vertex_iterator find(Point a);
        Vertex_iterator find_in(Point a);
        Vertex_iterator find_in(Point a, Segment& found_on);
        Vertex_iterator find_in_closest(Point a, double close, Segment& found_on);
        bool test_bounded_side( Polygon* master );
    };

}

#endif
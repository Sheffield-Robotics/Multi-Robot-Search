#ifndef GEOMETRY_UTIL
#define GEOMETRY_UTIL

#include "geometry_types.h"
#include "polygonization/polygon.h"

namespace polygonization { 

//Segment get_shortest_line(Segment l1, Segment l2);
Segment get_shortest_line(const Segment& l1, const Segment& l2);
Segment get_shortest_line(Polygon& a, Polygon& b);
Segment get_shortest_line(Polygon* a, Polygon* b);
Segment get_shortest_line(Polygon* a, Segment& b);
    
}

#endif
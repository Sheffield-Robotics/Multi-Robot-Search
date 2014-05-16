#include "polygonization/geometry_util.h"


namespace polygonization { 

Segment get_shortest_line(const Segment& l1, const Segment& l2) {
    
    Point l1Points[] = { l1.source(), l1.target() };
    Polygon_base l1Poly(l1Points,l1Points+2);
    Point l2Points[] = { l2.source(), l2.target() };
    Polygon_base l2Poly(l2Points,l2Points+2);
    
    Polytope_distance pd(l1Points, l1Points+2, l2Points, l2Points+2);
    assert (pd.is_valid());
    
    Polytope_distance::Coordinate_iterator coord_it;
    KERNEL::RT x,y,w;
    coord_it = pd.realizing_point_p_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p1(x,y,w);
    
    coord_it = pd.realizing_point_q_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p2(x,y,w);
    
    return Segment(p1,p2);
}

Segment get_shortest_line(Polygon& a, Polygon& b) {
    
    Polytope_distance pd(
        a.vertices_begin(), a.vertices_end(), 
        b.vertices_begin(), b.vertices_end() );

    assert (pd.is_valid());
    
    Polytope_distance::Coordinate_iterator coord_it;
    KERNEL::RT x,y,w;
    coord_it = pd.realizing_point_p_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p1(x,y,w);
    
    coord_it = pd.realizing_point_q_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p2(x,y,w);
    
    return Segment(p1,p2);
}

Segment get_shortest_line(Polygon* a, Polygon* b) {
    
    Polytope_distance pd(
        a->vertices_begin(), a->vertices_end(), 
        b->vertices_begin(), b->vertices_end() );

    assert (pd.is_valid());
    
    Polytope_distance::Coordinate_iterator coord_it;
    KERNEL::RT x,y,w;
    coord_it = pd.realizing_point_p_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p1(x,y,w);
    
    coord_it = pd.realizing_point_q_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p2(x,y,w);
    
    return Segment(p1,p2);
}

Segment get_shortest_line(Polygon* a, Segment& b) {

    Point l2Points[] = { b.source(), b.target() };
    Polygon_base l2Poly(l2Points,l2Points+2);
    
    Polytope_distance pd(
        a->vertices_begin(), a->vertices_end(), 
        l2Poly.vertices_begin(), l2Poly.vertices_end() );

    assert (pd.is_valid());
    
    Polytope_distance::Coordinate_iterator coord_it;
    KERNEL::RT x,y,w;
    coord_it = pd.realizing_point_p_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p1(x,y,w);
    
    coord_it = pd.realizing_point_q_coordinates_begin();
    x = *(coord_it++);
    y = *(coord_it++);
    w = (*coord_it);
    Point p2(x,y,w);
    
    return Segment(p1,p2);
}

}
#include "polygonization/polygon.h"
#include "polygonization/geometry_util.h"
#include <iostream>

namespace polygonization {
    
double Polygon::distance_to( Polygon* poly ) {
    if ( poly == this ) {
        std::cout << "SAME POLY" << std::endl;
        return 0.0;
    }
    if ( poly == NULL ) {
        std::cout << "NULL POLY" << std::endl;
        return -1.0;
    }
    
    double shortest_d = -1.0;
    Polygon::Vertex_iterator vertex_i = poly->vertices_begin();
    if ( vertex_i == poly->vertices_end() ) {
        return shortest_d;
    }
    
    Point last_point = *vertex_i;
    vertex_i++;
    while ( vertex_i != poly->vertices_end() ) {
        Segment s(last_point,*vertex_i);
        //std::cout << " s " << s << std::endl;
        Polygon::Vertex_iterator vertex_j;
        
        Segment short_s = polygonization::get_shortest_line(poly,s);
        //std::cout << " short_s " << short_s << std::endl;
        
        //// TODO: fix this (potentially)
        //vertex_j = this->vertices_begin();
        //Point last_point2 = *vertex_j;
        //if ( vertex_j == this->vertices_end() ) 
        //    return shortest_d;
        //vertex_j++;
        //while ( vertex_j != this->vertices_end() ) {
        //    Segment s2(last_point2, *vertex_j);
        //    //std::cout << " s2 " << s2 << std::endl;
        //    Segment short_s = polygonization::get_shortest_line(s2, s);
        //    double dist = CGAL::to_double( short_s.squared_length());
        //    if ( shortest_d > dist || shortest_d == -1 ) {
        //        shortest_d = dist;
        //    }
        //    last_point2 = *vertex_j;
        //    vertex_j++;
        //}
        
        double d = CGAL::to_double( short_s.squared_length());
        if ( shortest_d > d || shortest_d == -1 ) {
            shortest_d = d;
        }
        
        last_point = s.target();
        vertex_i++;
        
    }
    //std::cout << " distance_to - return " << shortest_d << std::endl;
    return sqrt(shortest_d);
}

void Polygon::print() {
    Polygon::Vertex_iterator vertex_i = this->vertices_begin();
    while ( vertex_i != this->vertices_end() ) {
        std::cout << *vertex_i << ", ";
        vertex_i++;
    }
    std::cout << std::endl;
}

Polygon::Vertex_iterator Polygon::find(Point a) {
    Polygon::Vertex_iterator vertex_i = this->vertices_begin();
    while ( vertex_i != this->vertices_end() ) {
        if ( *vertex_i == a ) {
            return vertex_i;
        }
        vertex_i++;
    }
    return vertex_i;
}

Polygon::Vertex_iterator Polygon::find_in(Point a) {
    Point lastPoint = this->vertex( this->size()-1 );
    Polygon::Vertex_iterator it = this->vertices_begin();
    if ( lastPoint == a ) {
        std::cout << " RETURNING end -- " << std::endl;
        it = this->vertices_end();
        it--;
        return it;
    }
    while ( it != this->vertices_end() ) {
        Segment s(lastPoint,*it);
        if ( s.has_on(a) ) {
            return it;
        }
        lastPoint = *it;
        it++;
    }
    return it;
}


Polygon::Vertex_iterator Polygon::find_in(Point a, Segment& found_on) {
    Point lastPoint = this->vertex( this->size()-1 );
    Polygon::Vertex_iterator it = this->vertices_begin();
    if ( lastPoint == a ) {
        std::cout << " RETURNING end -- " << std::endl;
        it = this->vertices_end();
        it--;
        found_on = Segment(this->vertex( this->size()-2 ),lastPoint);
        return it;
    }
    while ( it != this->vertices_end() ) {
        Segment s(lastPoint,*it);
        if ( s.has_on(a) ) {
            found_on = s;
            return it;
        }
        lastPoint = *it;
        it++;
    }
    return it;
}

Polygon::Vertex_iterator Polygon::find_in_closest(Point a, double close, Segment& found_on) {
    Point lastPoint = this->vertex( this->size()-1 );
    Polygon::Vertex_iterator it = this->vertices_begin();
    if ( lastPoint == a ) {
        std::cout << " RETURNING end -- " << std::endl;
        it = this->vertices_end();
        it--;
        found_on = Segment(this->vertex( this->size()-2 ),lastPoint);
        return it;
    }    
    double d = -1;
    while ( it != this->vertices_end() ) {
        Segment s(lastPoint,*it);
        d = CGAL::to_double( squared_distance( s, a) );
        if ( d < close ) {
            found_on = s;
            return it;
        }
        lastPoint = *it;
        it++;
    }
    return it;
}

bool Polygon::test_bounded_side( Polygon* master ) {
    for ( unsigned int i =0; i < this->size(); i++ ) {
        if ( master->bounded_side(this->vertex(i)) != CGAL::ON_BOUNDED_SIDE ) {
            return true;
        }
    }
    return false;
}

}
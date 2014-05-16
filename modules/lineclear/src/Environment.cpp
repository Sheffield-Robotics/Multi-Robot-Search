#include "lineclear/Environment.h"

using namespace lineclear;

Environment::Environment() {
    _sensorRange = 1;
}

Environment::Environment(Polygon* p) {
    _polygon = p;
    _sensorRange = 1;
    _arrangement = new Arrangement();
    CGAL::insert_non_intersecting_curves(*_arrangement, _polygon->edges_begin(), _polygon->edges_end());
    //_nn_tree = new NN_Tree( _polygon->vertices_begin(), 
    //    _polygon->vertices_end() );
}

Environment::Environment(Polygon* p, double r) {
    _polygon = p;
    _sensorRange = r;
    //_nn_tree = new NN_Tree( _polygon->vertices_begin(), 
    //    _polygon->vertices_end() );
}

int Environment::get_obstacle_number() {
    return _polygon->size();
}

Point Environment::get_closest_point(Point& a, Segment& s ) {
    // prepare line for intersection
    const Transformation rotate(CGAL::ROTATION, 
                                sin(CGAL_PI/2), 
                                cos(CGAL_PI/2));
    Vector normalV(s.source(), s.target());
    normalV = rotate(normalV);
    Line line(a, Direction(normalV) );
    
    // do the intersection
    CGAL::Object result = CGAL::intersection(s, line);
    if (const Point *ipoint = CGAL::object_cast<Point>(&result)) {
        return (*ipoint);
    } else if (const Segment *iseg = CGAL::object_cast<Segment>(&result)) {
        // line is parallel to segment 
        double d1 = CGAL::to_double(squared_distance(iseg->source(),a));
        double d2 = CGAL::to_double(squared_distance(iseg->target(),a));
        if ( d1 < d2 ) {
            return iseg->source();
        } else {
            return iseg->target();
        }
    } else {
        // no intersection between normal line through a and s
        // hence either target or source is closest
        // check which one is closer
        double d1 = CGAL::to_double(squared_distance(s.source(),a));
        double d2 = CGAL::to_double(squared_distance(s.target(),a));
        if ( d1 < d2 ) {
            return s.source();
        } else {
            return s.target();
        }
    }
}

int Environment::is_on_line( int i, int j, Point& p ) {
    this->fix_index(i);
    this->fix_index(j);
    Segment l1 = _polygon->edge(i-1);
    Segment l2 = _polygon->edge(j-1);
    if ( squared_distance(l1,p) <= squared_distance(l2,p) ) {
        return i;
    } else {
        return j;
    }
}

Segment Environment::get_shortest_line(int i, int j ) {
    Segment l1 = _polygon->edge(i-1);
    Segment l2 = _polygon->edge(j-1);
    if ( this->are_adjacent(i,j) ) {
        if ( ENVIRONMENT_DEBUG >= 2 ) 
            std::cout << " shortest line is a point " << std::endl;
        if (l1.source() == l2.target() )
            return Segment(l1.source(),l1.source()); 
        else if (l1.source() == l2.source() )
            return Segment(l1.source(),l1.source()); 
        else
            return Segment(l1.target(),l1.target()); 
    }
    
    Point l1Points[] = { l1.source(), l1.target() };
    Polygon l1Poly(l1Points,l1Points+2);
    Point l2Points[] = { l2.source(), l2.target() };
    Polygon l2Poly(l2Points,l2Points+2);
    
    Polytope_distance pd(l1Points, l1Points+2, l2Points, l2Points+2);
    assert (pd.is_valid());
    
    Polytope_distance::Coordinate_iterator coord_it;
    K::RT x,y,w;
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

Segment Environment::get_shortest_line_inside(int i, int j) {
    this->fix_index(i);
    this->fix_index(j);
    if ( ENVIRONMENT_DEBUG >= 2 )
        std::cout << " get_shortest_line_inside " << i <<"," << j << std::endl;
    Segment shortL = this->get_shortest_line(i,j);
    if ( shortL.target() == shortL.source() ) {
        return shortL;
    }
    if ( this->is_line_inside( shortL ) ) {
        return shortL;
    } else {
        // shortest line is not inside, check endpoint combinations
        double return_line_c = -1;
        bool have_return_line = false;
        Point a,b;
        Segment l, return_line;
        Segment l_i = _polygon->edge(i-1);
        Segment l_j = _polygon->edge(j-1);
        //std::cout << " l_i " << l_i << std::endl;
        //std::cout << " l_j " << l_j << std::endl;
        for ( int com = 0; com < 4; com++ ) {
            if ( com == 0 ) {
                a = _polygon->edge(i-1).source();
                l = _polygon->edge(j-1);;
            }
            else if ( com == 1 ) {
                a = _polygon->edge(i-1).target();
                l = _polygon->edge(j-1);;
            }
            else if ( com == 2 ) {
                a = _polygon->edge(j-1).source();
                l = _polygon->edge(i-1);;
            }
            else if ( com == 3 ) {
                a = _polygon->edge(j-1).target();
                l = _polygon->edge(i-1);;
            }
               
            b =  get_closest_point(a, l);
            // form a line from a to b
            Segment new_line(a,b);
            if ( ENVIRONMENT_DEBUG >= 3 )
                std::cout << " new_line " << new_line << std::endl;
            if ( is_line_inside( new_line ) ) {
                if ( ENVIRONMENT_DEBUG >= 3 )
                    std::cout << " is inside " << std::endl;
                double len = CGAL::to_double(new_line.squared_length());
                if ( return_line_c == -1 
                  || len < return_line_c ) {
                    return_line_c = len;
                    have_return_line = true;
                    return_line = new_line;
                }
            }
            
        }
        if ( have_return_line == false ) {
            if ( are_one_step_adjacent(i,j) ) {
                // TODO: there is still an issue here with 
                // segments that are are_one_step_adjacent but 
                // where the shortest line intersects a concavity
                // and the actual shortest line goes through the 
                // innermost point of said concavity

                // check the endpoints to the line segments
                int o;
                if ( -3 < i-j && i-j < 3 ) { 
                    if ( i < j ) 
                        o = i+1;
                    else
                        o = j+1;
                }
                if ( abs(i-j) >= int(_polygon->size())-2 ) { 
                    if ( i < j )
                        o = j+1;
                    else 
                        o = i+1;
                }
                fix_index(o);

                return _polygon->edge(o-1);
            }
            return Segment(Point(0,0),Point(0,0));
        }
        else
            return return_line;
    }
}

bool Environment::is_line_inside( Segment& line ) {
    std::list<CGAL::Object> zone_result;
    get_zone(line, zone_result);

    std::list<CGAL::Object>::iterator i = zone_result.begin();
    //std::cout << " _________ line inside? " << line << std::endl;
    //std::cout << " _________ zone_result " << zone_result.size() << std::endl;
    bool inside = true;
    while ( i != zone_result.end() ) {
        CGAL::Object obj = *i;        
        //Arrangement::Vertex_handle vh;
        //if ( CGAL::assign (vh, obj)) {
        //    std::cout << " ____ Vertex_handle ";
        //    std::cout << vh->point() << std::endl;
        //}
        //
        //Arrangement::Halfedge_handle eh;
        //if ( CGAL::assign (eh, obj)) {
        //    std::cout << " ____ Halfedge_handle ";
        //    std::cout << eh->source()->point() << " to ";
        //    std::cout << eh->target()->point() << std::endl;
        //}
        
        Arrangement::Face_handle fh;
        if ( CGAL::assign (fh, obj)) {
            //std::cout << " ____ Face_handle ";
            //std::cout << " is_unbounded?" << fh->is_unbounded();
            //std::cout << " has_outer_ccb?" << fh->has_outer_ccb();
            //std::cout << std::endl;
            if (  fh->is_unbounded() ) {
                inside = false;
                return false;
            }
        }
        i++;
    }
    return inside;
}

void Environment::get_zone(Segment& line, std::list<CGAL::Object>& res) {
    Arr_Segment curve( line.source(),line.target());
    CGAL::zone( *_arrangement, curve, std::back_inserter(res) );
}

int Environment::get_shortest_line_inside_cost(int i, int j) {
    this->fix_index(i);
    this->fix_index(j);
    if ( this->are_adjacent(i,j) ) { 
        return 0; 
    }
    
    Segment shortL = this->get_shortest_line_inside(i,j);    
    // shortest line is outside and their are not one step adjacent

    if ( shortL.target().x() == 0 &&
         shortL.target().y() == 0 && 
         shortL.source().x() == 0 &&
         shortL.source().y() == 0 ) {
        return -1;
    }
    double d = sqrt(CGAL::to_double(shortL.squared_length()));
    return round ( d / _sensorRange );
}

Segment Environment::get_line_inside(Segment const &line) {
    //std::cout << " get_line_inside " << std::endl;
    std::list<Polygon_wh> intR;
    this->intersect_line_with_poly(line,std::back_inserter(intR));
    if ( intR.size() == 1 ) {
        Polygon outer_boundary = intR.back().outer_boundary();
        if ( outer_boundary.has_on_boundary( line.source() )
          && outer_boundary.has_on_boundary( line.target() ) ) {
             return line;
        }
        return Segment(Point(0,0),Point(0,0));
    } else {
        // we have too few or too many polygon results
        return Segment(Point(0,0),Point(0,0));
    }
}

int Environment::get_line_inside_cost(Segment &line) {
    Segment s = line;
    if ( line.squared_length() > 0 ) {
        if ( this->is_line_inside(line) ) {
            double d = sqrt(CGAL::to_double(s.squared_length()));
            return round ( d / _sensorRange );
        } else {
            return -1;
        }
    } else if ( line.target() == Point(0,0) ){
        return -1;
    }
    return 0;
}

int Environment::get_line_cost( Segment &s ) {
    double d = sqrt(CGAL::to_double(s.squared_length()));
    return ceil ( d / _sensorRange );
}

int Environment::get_shortest_extension(int i, int j, int k, 
                                        Segment &l1, Segment &l2) {
    this->fix_index(i);
    this->fix_index(j);
    this->fix_index(k);
    
    Segment l_i = _polygon->edge(i-1);
    Segment l_j = _polygon->edge(j-1);
    Segment l_k = _polygon->edge(k-1);
    
    // TODO: a quick distance check to avoid costly computation of 
    // points below
    //if ( 
    //    sqrt( CGAL::to_double( squared_distance(l_i,l_k) ) )
    //  + sqrt( CGAL::to_double( squared_distance(l_j,l_k) ) )
    //  > Params::g_distance_sum_cutoff) {
    //    return -1;
    //}
    
    Vector v(l_k.source(), l_k.target());
    int c_i, c_j, c, max_c = -1;

    // TODO: this stuff has to be improved
    // TODO: binary search could be fast enough
    // TODO: analytic solution possible?    
    int n_iterations = 20;
    
    for ( int t = 0; t <= n_iterations; t++ ) {
        
        Point a = l_k.source() + (float (t)/ float (n_iterations)) * v;
        if ( t == n_iterations-1 ) {
            a = l_k.source() + 0.999999 * v;
        } else if ( t == n_iterations ) {
            a = l_k.target();
        } else if ( t == 0 ) {
            a = l_k.source();
        }

        Point b_i = this->get_closest_point(a, l_i);        
        Segment new_line_i(a,b_i);
        c_i = get_line_inside_cost( new_line_i );
        if ( c_i == -1 )
            continue;
        
        Point b_j = this->get_closest_point(a, l_j);
        Segment new_line_j(a,b_j);
        c_j = get_line_inside_cost( new_line_j );
        if ( c_i == -1 || c_j == -1 ) {
            continue;
        }
        
        c = c_i + c_j;
        
        if ( c <= max_c || max_c == -1 ) {
            max_c = c;
            l1 = new_line_i;
            l2 = new_line_j;
        }
    }
    return max_c;
}



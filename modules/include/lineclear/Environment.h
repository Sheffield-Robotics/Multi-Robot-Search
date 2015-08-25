    // 
//  Environment.h
//  icra12_pe_code
//  
//  Created by Andreas on 2012-08-28.
//  Copyright 2012 Andreas Kolling. All rights reserved.
// 

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#define ENVIRONMENT_DEBUG 1

#include "lineclear/CGALTypes.h"
#include "utilities/paramfile.h"

namespace lineclear 
{
    class Environment 
    {
      
      public:
        Environment();
        Environment(Polygon* p);
        Environment(Polygon* p, double r);
        int get_obstacle_number();    
        
        /* 
         * Returns the closest point on s to a
         * 
         * Author: Andreas Kolling ( Tue Aug 28 18:14:33 CEST 2012 )
         */
        Point get_closest_point(Point& a, Segment& s );
        
        /*
         * test whether point p is closer to segment i or j and returns
         * i or j respectively
         *
         * Author: Andreas Kolling ( Mon Oct  8 16:47:13 CEST 2012 ) 
         */
        int is_on_line( int i, int j, Point& p );
        
        /* 
         * Returns the shortest line between two poly edges i and j
         * 
         * Author: Andreas Kolling ( Tue Aug 28 18:22:16 CEST 2012 )
         */
        Segment get_shortest_line(int i, int j);
        
        /* 
         * Returns the shortest line between i and j if it is entirely inside
         * the environment, otherwise it returns the (0,0),(0,0) segment
         * 
         * Author: Andreas Kolling ( Tue Aug 28 17:30:59 CEST 2012)
         */
        Segment get_shortest_line_inside(int i, int j);
        int get_shortest_line_inside_cost(int i, int j);

        
        /* 
         * if we already know a line, we can return it and its cost if it is
         * entirely inside the polygon
         * 
         * Author: Andreas Kolling ( Tue Aug 28 18:39:21 CEST 2012 )
         */
        Segment get_line_inside(Segment const &line);
        int get_line_inside_cost(Segment &line);
        
        bool is_line_inside( Segment& line );
        void get_zone( Segment& line, std::list<CGAL::Object>& res);
        
        /* 
         * Returns the cost of the lowest cost lines from i to k and then to
         * j (if all lines are inside environment, otherwise returns -1)
         * 
         * Author: Andreas Kolling ( Tue Aug 28 18:40:27 CEST 2012 )
         */
        int get_shortest_extension_cost(int i, int j, int k) {
            Segment l1, l2;
            return this->get_shortest_extension(i,j,k, l1, l2);
        }
        int get_shortest_extension(int i, int j, int k, Segment &l1, 
            Segment &l2);
        

        
        
        /* 
         * Helper functions that are used to compute the lines intersected
         * with the polygon environment, i.e. inside lines
         * 
         * Author: Andreas Kolling ( Tue Aug 28 18:40:55 CEST 2012 )
         */
        //template<class OutputIterator> OutputIterator 
        //get_shortest_line_intersected(int i, int j, OutputIterator o);
        //template<class OutputIterator> OutputIterator 
        //intersect_line_with_poly(Segment const &line, OutputIterator o);
        template<class OutputIterator> OutputIterator 
        get_shortest_line_intersected(int i, int j, OutputIterator o) {
            Segment line = this->get_shortest_line(i,j);        
            return intersect_line_with_poly(line,o);
        }

        template<class OutputIterator> OutputIterator 
        intersect_line_with_poly(Segment const &line, OutputIterator o) {
            if ( line.squared_length() > 0 ) {
                const Transformation rotate(CGAL::ROTATION, sin(CGAL_PI/2), 
                    cos(CGAL_PI/2));
                Vector v(line.source(), line.target());
                v = 0.5 * v + 0.001 * rotate(v);
                //// Need this stupid point since CGAL::intersection
                //// has troubles with polyogns that are segments 
                //// and have COLLINEAR orientation
                //// TODO: get around this hack
                Point p3 = line.source() + v;
                Point lPoints[] = { line.target(), line.source(), p3 };
                Polygon lPoly(lPoints,lPoints+3);
                if( lPoly.is_clockwise_oriented() ) {
                    lPoly.reverse_orientation();
                }
                CGAL::intersection( (*_polygon), lPoly,  o);
                return o;
            } 
            return o;
        }
        
        int get_line_cost( Segment &s );
        
        inline void fix_index(int &i) {
            if ( _polygon->size() <= 0 ) { return; }
            while ( i < 1 ) {
                i = i + int(_polygon->size());
            }
            while ( i > int(_polygon->size()) ) {
                i = i - _polygon->size();
            }
        }
        
        Segment get_edge(int i) {
            if ( 0 < i && i <= this->get_obstacle_number() )
                return _polygon->edge(i-1);
            else 
                return Segment(Point(0,0),Point(0,0));
        }
        
        bool are_adjacent(int i, int j ) {
            if ( -2 < i-j && i-j < 2 ) { 
                return true; 
            }
            if ( abs(i-j) >= int(_polygon->size()) - 1 ) { 
                return true; 
            }
            return false;
        }
        
      private:
        Polygon* _polygon;
        //NN_Tree* _nn_tree;
        Arrangement* _arrangement;
        double _sensorRange;
        
        bool are_one_step_adjacent(int i, int j) {
            if ( -3 < i-j && i-j < 3 ) { 
                return true; 
            }
            if ( abs(i-j) >= int(_polygon->size())-2 ) { 
                return true; 
            }
            return false;
        }
    };
}
#endif // ENVIRONMENT_H 
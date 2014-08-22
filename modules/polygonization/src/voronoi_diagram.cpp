#include "polygonization/voronoi_diagram.h"

namespace polygonization {
    
    bool Voronoi_Diagram::construct( Polygon_Environment* e ) {
        // build the site vector
        std::vector< Site_2 > site_v;
        Site_2 a_site;
        for ( unsigned int i = 0; i < e->size(); i++ ) { 
            //M_INFO1("  Adding sites for poly %d \n",i);
            Polygon* poly = &(e->at(i));
            Polygon::Vertex_iterator vertex_i;
            vertex_i = poly->vertices_begin();
            Point P1 = poly->vertex( poly->size() - 1 );
            Point P2;
            while ( vertex_i != poly->vertices_end() ) {
                P2 = *vertex_i;
                a_site = a_site.construct_site_2( P1, P2 );
                Bad_Segment c = site_to_bad_segment(a_site);
                site_to_polygon[c] = i;
                //std::cout << a_site << std::endl;
                if ( a_site.is_defined() ) 
                    site_v.push_back(a_site);
                P1 = P2;
                vertex_i++;
            }
        }
        
        M_INFO2("We have %d sites for the Voronoi Diagram.\n",site_v.size());
        std::vector< Site_2 >::iterator site_it, site_it_end;
        site_it     = site_v.begin(); site_it_end = site_v.end();
        this->insert( site_it, site_it_end );
        this->insert( site_v.begin(), site_v.begin() );
        return true;
    }
    
    Segment*** Voronoi_Diagram::parse( double** distance_mat, 
        Polygon_Environment* poly_env) {
        M_INFO2("Parsing Voronoi Diagram\n");
        Vertex_iterator i = this->vertices_begin();
        int poly_size = poly_env->size();
        M_INFO2("Parsing Voronoi Diagram - poly size  %d\n",poly_size);
        Segment*** segment_mat = new Segment**[poly_env->size()];
        for ( unsigned int j = 0; j < poly_env->size(); j++ ) {
            segment_mat[j] = new Segment*[poly_env->size()];
            for ( unsigned int jj = 0; jj < poly_env->size(); jj++ ) {
                segment_mat[j][jj] = new Segment[3];
            }
        }
        int polies[3];
        while ( i != this->vertices_end() ) {
            for ( int j = 0; j <= 2; j++ ) {
                if ( i->site(j)->site().is_segment() ) {
                    Bad_Segment c = site_to_bad_segment(i->site(j)->site());
                    polies[j] = site_to_polygon[c];
                } else {
                    polies[j] = -1;
                }
            }
            for (int j = 2; j > 0; j--) {
                for ( int jj = 0; jj < j; jj++ ) {
                    if ( polies[j] != polies[jj] 
                        && polies[j] != -1 && polies[jj] != -1 )
                    {
                        int p1 = polies[j];
                        int p2 = polies[jj];
                        Segment short_s = polygonization::get_shortest_line(
                            i->site(j)->site().segment(),
                            i->site(jj)->site().segment() );
                        double d = CGAL::to_double(short_s.squared_length());
                        
                        if ( p1 < p2 ) { p2 = p1; p1 =  polies[jj]; }
                        
                        if ( distance_mat[p1][p2] == -1 
                          || d < distance_mat[p1][p2] 
                          || d == distance_mat[p1][p2] ) {
                            if ( distance_mat[p1][p2] == d ) {
                                // check angles
                                // TODO: this could fix some stuff
                                // with tricky connections
                            } else {
                                std::cout << " setting segment ";
                                std::cout << p1 << " to " << p2;
                                std::cout << " d=" << d;
                                std::cout << " short_s=" << short_s;
                                std::cout << " s1=" 
                                    << i->site(j)->site().segment();
                                std::cout << " s2=" 
                                    << i->site(jj)->site().segment();
                                std::cout << std::endl;
                                distance_mat[p1][p2] = d;
                                segment_mat[p1][p2][0] = short_s;
                                segment_mat[p1][p2][1] = 
                                    i->site(j)->site().segment();
                                segment_mat[p1][p2][2] = 
                                    i->site(jj)->site().segment();
                            }
                        }
                    }
                }
            }
            i++;
        }
        return segment_mat;
    }
    
    Voronoi_Diagram::Bad_Segment 
    Voronoi_Diagram::site_to_bad_segment( Voronoi_Diagram::Site_2 s ) {
        Bad_Point a( CGAL::to_double(s.source().x()),
            CGAL::to_double(s.source().y()) );
        Bad_Point b( CGAL::to_double(s.target().x()),CGAL::to_double(s.target().y()) );
        Bad_Segment c( a,b);
        return c;
    }
    
}
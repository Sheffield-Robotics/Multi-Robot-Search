#include "polygonization/alpha_shape.h"

namespace polygonization {

    void Alpha_shape::construct( bool** occ, int size_x, int size_y, float a ) {
        _occupied = occ;
        _size_x = size_x;
        _size_y = size_y;
        _occ_points.clear();        
        for ( int x = 0; x < _size_x; ++x ) {
            for ( int y = 0; y < _size_y; ++y ) {
                if ( _occupied[x][y] ) {
                    _occ_points.push_back( Point(x,y) );
                }
            }
        }
        //this->set_mode( Alpha_shape_base::REGULARIZED );
        make_alpha_shape( _occ_points.begin(), _occ_points.end() );
        this->set_alpha( a );
        this->set_mode( Alpha_shape_base::REGULARIZED );
        //std::cout << "MODE" << this->get_mode() << std::endl;
        fflush(stdout);
    }
    
    Alpha_shape::vertex_handle 
    Alpha_shape::get_neighbor_vertex( edge_circulator e, vertex_handle v )
    {
        vertex_handle v1 = e->first->vertex( this->cw(e->second) );
        vertex_handle v2 = e->first->vertex( this->ccw(e->second) );
        if ( v1 != v && v2 != v )
            std::cout << " CRITICAL ERROR: edge not between vertices" 
                      << std::endl;
        return ( ( v1 == v )? v2:v1 );
    }

    bool Alpha_shape::is_regular( edge_circulator e )
    {
        return (this->classify( *e ) == Alpha_shape_base::REGULAR);
    }

    bool Alpha_shape::is_regular( edge_iterator e )
    {
        return (this->classify( *e ) == Alpha_shape_base::REGULAR);
    }

    int Alpha_shape::regular_edges( vertex_handle v )
    {
        edge_circulator e_circ1,e_circ2;
        e_circ1 = this->incident_edges( v );
        e_circ2 = e_circ1;
        int reg_edges = 0;
        do {
            if ( is_regular( e_circ1 ) ) {
                ++reg_edges;
            }
            ++e_circ1;
        } while ( e_circ1 != e_circ2 );
        return reg_edges;
    }

}
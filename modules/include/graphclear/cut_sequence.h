#ifndef GRAPHCLEAR_CUT_SEQUENCE_H
#define GRAPHCLEAR_CUT_SEQUENCE_H

#include <deque>

#include "graphclear/cut.h"
//#include "graphclear/surveillance_graph.h"


namespace graphclear
{
    
    class surveillance_graph_t;


class cut_sequence_t : public std::deque<cut_t>
{
public:
    int length;
    std::deque<surveillance_graph_t::vertex_descriptor> vertex_sequence;

    cut_sequence_t ();
    ~cut_sequence_t ();
    
    void 
        add( surveillance_graph_t::vertex_descriptor& v, 
            surveillance_graph_t& g);
    
private:
    /* data */
};

} /* cut_sequence */

#endif
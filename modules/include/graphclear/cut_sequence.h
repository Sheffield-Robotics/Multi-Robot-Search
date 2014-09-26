#ifndef GRAPHCLEAR_CUT_SEQUENCE_H
#define GRAPHCLEAR_CUT_SEQUENCE_H

#include <deque>
#include <algorithm>
#include <iostream>

#include "graphclear/cut.h"
#include "graphclear/sg_typedefs.h"


namespace graphclear
{
    class surveillance_graph_t;

class cut_sequence_t : public std::deque<cut_t>
{
public:
    int length;
    std::deque<sg_base::vertex_descriptor> vertex_sequence;
    std::set<cut_t> ordered_cuts;

    cut_sequence_t ();
    ~cut_sequence_t ();
    
    void add_cut( cut_t y );
    void add_cut_unordered( cut_t y );
    void 
        add( sg_base::vertex_descriptor v, int w, int w_v,
            surveillance_graph_t& g);
    void make_full();
private:
    /* data */
};

} /* cut_sequence */

#endif
#include "graphclear/cut_sequence.h"

using namespace graphclear;

cut_sequence_t::cut_sequence_t()
{
    length = 0;
}

cut_sequence_t::~cut_sequence_t()
{
    
}

void cut_sequence_t::add( 
    sg_base::vertex_descriptor v, 
    surveillance_graph_t& g)
{
    vertex_sequence.push_back(v);
}
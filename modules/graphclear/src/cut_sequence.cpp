#include "graphclear/cut_sequence.h"

using namespace graphclear;

cut_sequence_t::cut_sequence_t()
{
    length = 0;
}

cut_sequence_t::~cut_sequence_t()
{
    
}

void cut_sequence_t::add_cut( cut_t y ) 
{
    ordered_cuts.insert(y);
}

void cut_sequence_t::add_cut_unordered( cut_t y ) 
{
    this->push_back(y);
}

void cut_sequence_t::add( 
    sg_base::vertex_descriptor v, 
    int w, int w_v,
    surveillance_graph_t& g)
{       
    vertex_sequence.push_back(v);

    cut_t y;
    if ( this->size() > 0 ) {
        y = this->back();
        y.push_back(v);
        y.ag = std::max(w_v,y.ag);
        y.b = w;
        y.rho = y.ag - this->back().b;
    } else {
        y.push_back(v);
        y.ag = w_v;
        y.b = w;
    }
    this->push_back(y);
}

/*
    Turns the ordered_cuts into a full cut sequence
*/ 
void cut_sequence_t::make_full() 
{
    //ordered_cuts.
    
        
}
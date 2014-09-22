#include "graphclear/surveillance_graph.h"

using namespace graphclear;

void surveillance_graph_t::cut_strategy()
{
    std::deque< surveillance_graph_t::vertex_descriptor > q;
    
    // add all leaves to the queue
    surveillance_graph_t::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(*this); vi != vi_end; ++vi)
    {
        if ( out_degree(*vi,*this) == 1 ) 
            q.push_back(*vi);
    }
    
    while ( ! q.empty() )
    {
        surveillance_graph_t::vertex_descriptor v_y, v_x;
        v_y = q.front();
        q.pop_front();
        
        if ( out_degree(v_y,*this) == 1 ) {
            
        } else {
            if ( (*this)[v_y].outgoing_completed >= out_degree(v_y,*this)-1 ) 
            {
                // we have a sufficient number of outgoing 
                // cut sequences so that we can build an incoming 
                // cut sequence
                
                surveillance_graph_t::out_edge_iterator ei, ei_end;
                
                boost::tie(ei, ei_end) = out_edges(v_y,*this);
                for ( ; ei != ei_end; ++ei ) 
                {
                    cut_sequence_t cut_s = this->construct_full_cut_sequence();
                    
                    v_x = this->get_other(ei,v_y);
                }
            }
        }
            
    }
    
}

graphclear::cut_sequence_t 
surveillance_graph_t::construct_full_cut_sequence()
{
    graphclear::cut_sequence_t s;
    return s;
}

int surveillance_graph_t::count_outgoing_sequences()
{
    return 1;
}

surveillance_graph_t::vertex_descriptor 
surveillance_graph_t::get_other( out_edge_iterator ei, vertex_descriptor v)
{
    if ( boost::source(*ei,*this) == v )
        return boost::target(*ei,*this);
    else 
        return boost::source(*ei,*this);
}

int main (int argc, char const *argv[])
{
    /* code */
    return 0;
}
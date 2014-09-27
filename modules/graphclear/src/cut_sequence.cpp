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
    int w, int ag,
    surveillance_graph_t& g)
{       
    vertex_sequence.push_back(v);

    cut_t y;
    if ( this->size() > 0 ) {
        y = this->back();
        y.push_back(v);
        y.ag = std::max(ag,y.ag);
        y.b = w;
        y.rho = y.ag - this->back().b;
    } else {
        y.push_back(v);
        y.ag = ag;
        y.b = w;
    }
    this->push_back(y);
}

/*
    Turns the ordered_cuts into a full cut sequence
*/ 
void cut_sequence_t::make_full() 
{
    std::cout << std::endl << "Making full " << std::endl;
    iterator it = this->begin(); // it through cuts
    iterator old_it, best_it;
    int b = it->b;
    int best_b;
    int ag = it->ag;
    int ag_max = ag;
    std::cout << "First cut " << std::endl << *it << std::endl;
    it++; // first cut stays
    bool done = false;
    std::cout << "Next cutc " << std::endl;
    while ( done == false && it != this->end() ) 
    {
        std::cout << *it << std::endl;
        if ( ag_max < it->ag ) 
            ag_max = it->ag;
        it->ag = ag_max;
        if ( it->b < b )
        {
            std::cout << " Cut has better blocking at cost " << ag_max 
                << std::endl;    
            // found a lower blocking cost, go further at the same cost
            // in case we find a lower one that can be reached as well
            old_it = it;
            best_b = it->b;
            best_it = it;
            it++;
            while ( it != this->end() && it->ag <= ag_max ) {
                if ( it->b < best_b ) {
                    // found a better cut at same cost
                    std::cout << " Cut has even better blocking " << std::endl;
                    this->erase(best_it);
                    best_it = it;
                    best_b = it->b;
                    best_it->ag = ag_max;
                    std::cout << *it << std::endl;
                    it++;
                    if ( it == this->end() )
                        done = true;
                } else {
                    // cut is not better, but cost the same, remove
                    std::cout << " Cut has worse blocking - del" << std::endl;
                    it = this->erase(it);
                }
            }
            // now best_it is the next lowest blocking cost reachable at
            // ag_max
            b = best_b;
            std::cout << " New best b " << best_b << std::endl;
        } else {
            std::cout << " Cut has worse blocking - at " << ag_max << std::endl;    
            it = this->erase(it);
        }
        
    }
    
    std::cout << std::endl << " *** full cut sequence *** " << std::endl;
    it = this->begin();
    std::cout << *it << std::endl; 
    int last_b = it->b;
    it++;
    while ( it != this->end() ) {
        it->rho = it->ag - last_b;
        std::cout << *it << std::endl; 
        it++;
    }
    
    std::cout << std::endl << std::endl;
        
}
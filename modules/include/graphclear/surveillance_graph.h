#ifndef GRAPHCLEAR_SURVEILLANCE_GRAPH_H
#define GRAPHCLEAR_SURVEILLANCE_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/small_world_generator.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/graph/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/astar_search.hpp>

#include <climits>
#include <fstream>

#include "graphclear/sg_typedefs.h"
#include "graphclear/cut_sequence.h"

namespace graphclear
{
    
    
class surveillance_graph_t : public sg_base
{
public:
    surveillance_graph_t(){};
    
    template<class EdgeIterator>
    surveillance_graph_t(EdgeIterator first, EdgeIterator last,
                          vertices_size_type n)
            : sg_base(first, last, n)    // Call the superclass constructor in the subclass' initialization list.
            {
                // do something with bar
            };

    
    void cut_strategy();
    
    void
    construct_full_cut_sequence(vertex_descriptor from, vertex_descriptor to);
    
    int count_outgoing_sequences();
    
    graphclear::cut_sequence_t*
    get_cut_sequence( 
        vertex_descriptor from, vertex_descriptor to);
    
    vertex_descriptor get_other( out_edge_iterator ei, vertex_descriptor v);

    cut_sequence_t* find_best_strategy();
    cut_sequence_t* best_strategy_at_vertex( vertex_descriptor v);
    int play_through_strategy(cut_t& strategy, std::string filename = "");
    
    void
    print_graph_to_txt_file(const char* filename);
    
    float distance_between( vertex_descriptor v, vertex_descriptor w );
    float distance_between( vertex_descriptor v, edge_descriptor e );
    float distance_between( edge_descriptor ee, edge_descriptor e );

};

void write_tree_to_file(surveillance_graph_t& tree_of_g);
void graph_to_tree(surveillance_graph_t& g, surveillance_graph_t& tree_of_g);
void gen_rand_graph(surveillance_graph_t& g, int nV, int nE, int min_v_w,int max_v_w, int min_e_w, int max_e_w);
 void gen_rand_physical_graph(surveillance_graph_t& g, double grid, int nV, int min_v_w,int max_v_w, int min_e_w, int max_e_w,double connect_thres, double all_connect_d);

void cleanup_tree(surveillance_graph_t& tree_of_g);


} /* graphclear */

#endif

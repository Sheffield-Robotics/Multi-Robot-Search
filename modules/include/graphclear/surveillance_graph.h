#ifndef GRAPHCLEAR_SURVEILLANCE_GRAPH_H
#define GRAPHCLEAR_SURVEILLANCE_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/small_world_generator.hpp>
#include <boost/random/linear_congruential.hpp>

#include "graphclear/cut_sequence.h"

namespace graphclear
{

class surveillance_graph_vertex
{
public:
    int w;
    
    int outgoing_completed;
};

class surveillance_graph_edge
{
public:
    cut_sequence_t cut_sequence_source_to_target;
    cut_sequence_t cut_sequence_target_to_source;
    int w;
    int edge_weight;
};

typedef boost::adjacency_list
    <boost::vecS, boost::vecS, boost::undirectedS, 
     surveillance_graph_vertex, surveillance_graph_edge >
    sg_base;

class surveillance_graph_t : public sg_base
{
public:
    surveillance_graph_t();
    
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
    

    graphclear::cut_sequence_t&
    get_cut_sequence( 
        vertex_descriptor from, vertex_descriptor to);
    
    vertex_descriptor get_other( out_edge_iterator ei, vertex_descriptor v);

};

} /* graphclear */

#endif
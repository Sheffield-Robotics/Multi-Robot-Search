#ifndef SVG_GRAPH
#define SVG_GRAPH

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "svg_vertex.h"
#include "svg_edge.h"
#include "define.h"

typedef adjacency_list<vecS, vecS, undirectedS, svg_vertex, svg_edge >
svg_graph_base;
typedef graph_traits<svg_graph_base>::vertex_descriptor
svg_vertex_d;
typedef graph_traits<svg_graph_base>::edge_descriptor
svg_edge_d;
typedef graph_traits<svg_graph_base>::vertex_iterator
svg_vertex_it;
typedef graph_traits<svg_graph_base>::edge_iterator
svg_edge_it;
typedef graph_traits<svg_graph_base>::out_edge_iterator
svg_o_edge_it;
typedef graph_traits<svg_graph_base>::adjacency_iterator
svg_adj_it;
typedef pair< svg_edge_d, bool>
edge_bool;


class svg_graph : public svg_graph_base
{
public:
    int nVertices;

    svg_graph();

    void
    parse_dot_file( const char* filename );

    svg_vertex_d
    get_first( svg_edge_d ed);
    svg_vertex_d
    get_second( svg_edge_d ed);
    int
    get_label( svg_vertex_d vd, svg_edge_d ed, bool outbound);
    svg_vertex_d
    get_neighbor( svg_vertex_d vd, svg_edge_d e);
    svg_vertex_d
    get_random_vd( );
    void
    print_graph_to_file(const char* filename, bool non_mst_edges);

};

#endif

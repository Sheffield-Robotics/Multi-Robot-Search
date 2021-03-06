#ifndef SVG_LABEL_COMPUTER
#define SVG_LABEL_COMPUTER

#include "define.h"
#include "svg_graph.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>

using namespace std;
/* The label computer class computes the labels on the edges of an svg_graph */

typedef std::deque< svg_edge_d >     edge_deque;
typedef size_t                       size_type;
typedef pair< svg_edge_d, int >      sorting_edge;
typedef std::vector< sorting_edge >  edge_vector;
typedef std::back_insert_iterator<std::vector<svg_edge_d> >
dfs_back_it_t;

class svg_label_computer
{
private:
    edge_deque edge_pipeline;
    std::vector<svg_edge_d> MST_edges;
    int max_traversal;
    void clean_up_graph(svg_graph& SVG);

    void
    convert_graph_to_tree( svg_graph& SVG, svg_vertex_d root_v );

    void
    dfs_spanning_tree( svg_graph& SVG, dfs_back_it_t dfs_edges ,
                       svg_vertex_d v_start ) ;
    void
    dfs_visit( svg_graph& SVG, dfs_back_it_t dfs_edges ,svg_vertex_d v );

    void leaf_edge_pipeline_fill(svg_graph& SVG);
    int get_mst_degree( svg_vertex_d vd, svg_graph& SVG );
    void pipeline_traverse(svg_graph& SVG);
    int edge_check_validity( svg_edge_d ed, svg_graph& SVG );
    svg_vertex_d
    get_first( svg_edge_d ed, svg_graph& SVG);
    svg_vertex_d
    get_second( svg_edge_d ed, svg_graph& SVG);
    bool
    out_going_label_computed( svg_vertex_d vd, svg_edge_d ed,
                              svg_graph& SVG);
    void
    compute_label( svg_edge_d ed, int di, svg_graph& SVG );
    int
    get_label( svg_vertex_d vd, svg_edge_d ed, bool outbound,
               svg_graph& SVG );
    int
    collect_neighbors( svg_vertex_d vd, svg_edge_d ed, edge_vector& e_n, svg_graph& SVG);

    int
    start_cost( svg_vertex_d vd, svg_graph& SVG);

public:
    svg_label_computer();
    void
    compute_labels( svg_graph& SVG,  svg_vertex_d root_v );
    svg_vertex_d
    get_best_start_vd( int& min_cost, svg_graph& SVG );

};

bool edge_comparison (const sorting_edge& a, const sorting_edge& b);
#endif

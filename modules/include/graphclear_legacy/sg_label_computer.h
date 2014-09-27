#ifndef SG_LABEL_COMPUTER
#define SG_LABEL_COMPUTER

#include "graphclear_legacy/define.h"
#include "graphclear_legacy/sg_graph.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>

using namespace std; 
/* The label computer class computes the labels on the edges of an sg_graph */  

typedef std::deque< sg_edge_d >     edge_deque;
typedef size_t                      size_type;
typedef pair< sg_edge_d, int >      sorting_edge;
typedef std::vector< sorting_edge > edge_vector;

class sg_label_computer { 
private:
  edge_deque edge_pipeline;
  std::vector<sg_edge_d> MST_edges;                                   
  int max_traversal;                     
  void clean_up_graph(sg_graph& SG);
  void convert_graph_to_tree(sg_graph& SG);
  void compute_save_sweep(sg_graph& SG);
  void leaf_edge_pipeline_fill(sg_graph& SG);
  int get_mst_degree( sg_vertex_d vd, sg_graph& SG );
  void pipeline_traverse(sg_graph& SG, int l_id);
  int edge_check_validity( sg_edge_d ed, int l_id, sg_graph& SG );
  sg_vertex_d 
    get_first( sg_edge_d ed, sg_graph& SG);
  sg_vertex_d 
    get_second( sg_edge_d ed, sg_graph& SG);
  bool 
    out_going_label_computed( sg_vertex_d vd, sg_edge_d ed, int l_id,
                                 sg_graph& SG);
  void 
    compute_label( sg_edge_d ed, int di, int l_id, sg_graph& SG );
  void 
    set_is_in_V_2( sg_vertex_d vd, sg_edge_d ed, bool value, 
                      sg_graph& SG );
  int 
    get_label( sg_vertex_d vd, sg_edge_d ed, int l_id, bool outbound,
                 sg_graph& SG );
  int 
    collect_neighbors( sg_vertex_d vd, sg_edge_d ed, edge_vector& e_n,
                         int l_id, sg_graph& SG);

  int 
    find_max_cost( int& max_i, int& max_l, edge_vector& n_e, 
  				   int n_e_size, sg_graph& SG);
  int 
    find_max_cost_hybrid( int& max_i, edge_vector& n_e, int penalty_w,
   						  int n_n_e, sg_graph& SG);
  int 
    partition_hybrid_h( edge_vector& neighbors, 
  					   int n_neigh, int penalty, int max_k, int max_i,
  					   int max_lambda, sg_graph& SG);
  int 
    partition_hybrid_t( edge_vector& n_e, sg_vertex_d vd, int n_n_e, 
  					    int p_w, int best, sg_graph& SG);      
  int 
    start_cost( sg_vertex_d vd, int l_id, sg_graph& SG);

public:
  sg_label_computer();
  void 
    compute_labels( sg_graph& SG, int l_id );
  sg_vertex_d 
    get_best_start_vd( int& min_cost, int l_id, sg_graph& SG );
  
};    

bool edge_comparison (const sorting_edge& a, const sorting_edge& b);
#endif

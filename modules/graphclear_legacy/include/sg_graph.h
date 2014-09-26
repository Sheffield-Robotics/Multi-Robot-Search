#ifndef SG_GRAPH
#define SG_GRAPH

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "sg_vertex.h"
#include "sg_edge.h"
#include "define.h"

using namespace std;
using namespace boost;

/*
 * Basic typedefs to create the structure for our surveillance
 * graphs from the BOOST graph library
 */
typedef adjacency_list<vecS, vecS, undirectedS, sg_vertex, sg_edge >
  sg_graph_base;
typedef graph_traits<sg_graph_base>::vertex_descriptor
  sg_vertex_d;
typedef graph_traits<sg_graph_base>::edge_descriptor
  sg_edge_d;
typedef graph_traits<sg_graph_base>::vertex_iterator
  sg_vertex_it;
typedef graph_traits<sg_graph_base>::edge_iterator
  sg_edge_it;
typedef graph_traits<sg_graph_base>::out_edge_iterator
  sg_o_edge_it;
typedef graph_traits<sg_graph_base>::adjacency_iterator
  sg_adj_it;
typedef pair< sg_edge_d, bool>
  edge_bool;

class sg_graph : public sg_graph_base {
  public:
	sg_graph();
	int label_id;
	void 
	  print_vertices();
	void 
	  print_graph_to_file(const char* filename, int l_id, bool non_mst_edges );
	void 
	  copy_edge_values( sg_edge_d ed1, sg_edge_d ed2 ); 
	sg_vertex_d 
	  get_first( sg_edge_d ed);
	sg_vertex_d 
	  get_second( sg_edge_d ed);
	int 
	  get_label( sg_vertex_d vd, sg_edge_d ed, int l_id, bool outbound);
	sg_vertex_d 
	  get_neighbor( sg_vertex_d vd, sg_edge_d e);
	float
	  distance_to_edge( sg_vertex_d vd1, sg_vertex_d vd2, int x, int y );
	int 
		get_sweep_cost( sg_vertex_d vd1, sg_vertex_d vd2);
	int 
		get_sweep_dir_index( sg_vertex_d vd1, sg_vertex_d vd2);
	int
		number_of_nonmst_edges( int& cost );
		
};

#endif

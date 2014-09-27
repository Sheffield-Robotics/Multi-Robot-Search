#ifndef SG_STRATEGY
#define SG_STRATEGY

#include <boost/graph/adjacency_list.hpp>

#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "graphclear_legacy/sg_graph.h"
#include "graphclear_legacy/sg_label_computer.h"

using namespace std;
using namespace boost;

typedef std::vector<sg_edge_d> sg_edge_vector;
   
class sg_strategy_step {
public:
  sg_edge_vector  new_blocks;
  sg_edge_vector  freed_blocks;
  sg_vertex_d     new_vertex;
};

typedef std::vector<sg_strategy_step>
  sg_strategy_seq;


/* 
 * sg_strategy contains a vector
 * 
 */

class sg_strategy {
private:
	sg_strategy_seq sequence;
	int             private_debug;
	int             l_id;
  
  edge_vector 
    collecting_neighbors( sg_graph* the_graph, sg_vertex_d current_v, sg_edge_d current_e, bool starting);
  void 
    check_mst_edge_blocks( sg_graph* the_graph, sg_vertex_d vd, sg_edge_d current_e, sg_strategy_step& new_step );
  void 
    visit_vertex( sg_graph* the_graph, sg_vertex_d current_v, sg_edge_d current_e, bool V2_vertex,   bool starting, int label_id);    
  bool 
    already_in( sg_vertex_d vd );
  
  
  
public:
  sg_strategy( sg_graph* the_graph , sg_vertex_d start_v, int label_id );
  
  void
    cout_strategy();
  
  sg_vertex_d
    get_vertex( int step );

	sg_vertex_d
		get_vertex( sg_graph* the_graph, int step, sg_vertex_d& from );
	

  int
    size();

	sg_strategy_step* 
		get_step( int step );

};

#endif

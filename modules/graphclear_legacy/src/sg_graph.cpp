#include "graphclear_legacy/sg_graph.h"

sg_graph::sg_graph() {
  label_id = 4;
}

void sg_graph::print_vertices() {
	sg_vertex_d 	vert_desc;
	sg_vertex_it	vert_it, vert_it_end;
	//  to traverse edges
	sg_o_edge_it 	out_edge_it, out_edge_it_end;
	sg_adj_it 	adj_it, adj_it_end;
	cout << "THE GRAPH " << endl;
	tie(vert_it, vert_it_end) = vertices(*this);
	for (; vert_it != vert_it_end; ++vert_it) {
		tie(out_edge_it, out_edge_it_end) 	= out_edges(*vert_it, *this);
		tie(adj_it, adj_it_end)  			= adjacent_vertices(*vert_it, *this);

		vert_desc = vertex(*vert_it, *this);
		cout << "    ";
		for (;out_edge_it != out_edge_it_end; out_edge_it++) {
			cout << *out_edge_it << "  ";
		}
		cout << " Vertex " << vert_desc << " mst_d="  << (*this)[vert_desc].mst_degree;
		if (adj_it == adj_it_end)
		  cout << " has no children";
		else
		  cout << " is connected to ";
		for (; adj_it != adj_it_end; adj_it++) {
			cout << *adj_it << ",";//<< (*this)[*ai].w;
		}
		cout << endl;
	}
}

bool sg_graph::edge_alive( sg_edge_d e ) 
{
    if ( (*this)[source(e,*this)].alive && (*this)[target(e,*this)].alive) {
        return true;
    } else {
        return false;
    }
}

void sg_graph::print_graph_to_file_txt(const char* filename) {
	ofstream out_file;
	sg_o_edge_it 	out_edge_it, out_edge_it_end;
	sg_adj_it 	adj_it, adj_it_end;
	sg_edge_it e_it, e_it_end;
    sg_vertex_it	vert_it, vert_it_end, v_it, v_it_end;
    sg_edge_it tree_edge_it,tree_edge_it_end;

	// Prepare the file into which to write the graph
	out_file.open(filename);//formerly out_file

    int nvertices = 0;
	tie(vert_it, vert_it_end) = vertices(*this);
	for (; vert_it != vert_it_end ; ++vert_it ) {
        if ( !(*this)[*vert_it].alive )
            continue;
		nvertices++;
	}
    int nedges = 0;
	tie( tree_edge_it, tree_edge_it_end ) = edges( *this );
	for (; tree_edge_it != tree_edge_it_end; tree_edge_it++) {
        if ( !edge_alive(*tree_edge_it) )
            continue;
		nedges++;
	}

    out_file << nvertices << ", " << nedges << std::endl;
    out_file << std::endl;
	tie(vert_it, vert_it_end) = vertices(*this);
	for (; vert_it != vert_it_end ; ) {
        if ( !(*this)[*vert_it].alive ) {
            vert_it++;
            continue;
        }
		out_file << (*this)[*vert_it].w;
        vert_it++;
        if ( vert_it == vert_it_end ) {
            out_file << std::endl;
        } else {
            out_file << ", ";
        }
	}
    out_file << std::endl;
    
	//  Edges
	tie( tree_edge_it, tree_edge_it_end ) = edges( *this );
	for (; tree_edge_it != tree_edge_it_end;) {
        if ( !edge_alive(*tree_edge_it) ) {
            tree_edge_it++;
            continue;
        }
		out_file << (*this)[*tree_edge_it].w;
        tree_edge_it++;
        if ( tree_edge_it == tree_edge_it_end ) {
            out_file << std::endl;
        } else {
            out_file << ", ";
        }
	}
    out_file << std::endl;

    //distance matrix n+m x n+m
	tie(vert_it, vert_it_end) = vertices(*this);
	for (; vert_it != vert_it_end ; ++vert_it ) {
        if ( !(*this)[*vert_it].alive )
            continue;
    	tie(v_it, v_it_end) = vertices(*this);
    	for (; v_it != v_it_end ; ++v_it ) {
            if ( !(*this)[*v_it].alive )
                continue;
    		out_file << this->distance_between(*vert_it,*v_it);
            out_file << ", ";
    	}
    	tie( e_it, e_it_end) = edges( *this );
    	for (; e_it != e_it_end; ) {
            if ( !edge_alive(*e_it) ) {
                e_it++;
                continue;
            }
    		out_file << this->distance_between(*vert_it,*e_it);
            e_it++;
            if ( e_it == e_it_end ) {
                out_file << std::endl;
            } else {
                out_file << ", ";
            }
    	}
	}
	tie( tree_edge_it, tree_edge_it_end ) = edges( *this );
	for (; tree_edge_it != tree_edge_it_end; tree_edge_it++) {
        if ( !edge_alive(*tree_edge_it) )
            continue;
    	tie(v_it, v_it_end) = vertices(*this);
    	for (; v_it != v_it_end ; ++v_it ) {
            if ( !(*this)[*v_it].alive )
                continue;
    		out_file << this->distance_between(*v_it,*tree_edge_it);
            out_file << ", ";
    	}
    	tie( e_it, e_it_end) = edges( *this );
    	for (; e_it != e_it_end; ) {
            if ( !edge_alive(*e_it) ){
                e_it++;
                continue;
            }
    		out_file << this->distance_between(*tree_edge_it,*e_it);
            e_it++;
            if ( e_it == e_it_end ) {
                out_file << std::endl;
            } else {
                out_file << ", ";
            }
    	}
    }
    
    // vertex to edge adjacency
	tie(vert_it, vert_it_end) = vertices(*this);
	for (; vert_it != vert_it_end ; ++vert_it ) {
        if ( !(*this)[*vert_it].alive )
            continue;
    	tie( e_it, e_it_end) = edges( *this );
    	for (; e_it != e_it_end; ) {
            if ( !edge_alive(*e_it) ) {
                e_it++;
                continue;
            }
    	    if ( source(*e_it,*this) == *vert_it 
                || target(*e_it,*this) == *vert_it) {
                out_file << "1";
            } else {
                out_file << "0";
            }
            e_it++;
            if (e_it == e_it_end )
                out_file << std::endl;
            else
                out_file << ", ";
    	}
    }
    out_file << std::endl;
    
    // edge to vertex adjacency
	tie( e_it, e_it_end) = edges( *this );
	for (; e_it != e_it_end; e_it++) {
        if ( !edge_alive(*e_it) )
            continue;
    	tie(vert_it, vert_it_end) = vertices(*this);
    	for (; vert_it != vert_it_end ;) {
            if ( !(*this)[*vert_it].alive ) {
                vert_it++;
                continue;
            }
            if ( source(*e_it,*this) == *vert_it 
                || target(*e_it,*this) == *vert_it) {
                out_file << "1";
            } else {
                out_file << "0";
            }
            vert_it++;
            if (vert_it == vert_it_end )
                out_file << std::endl;
            else
                out_file << ", ";
        }
	}
    out_file << std::endl;
    
    // vertex to vertex adjacency
	tie(vert_it, vert_it_end) = vertices(*this);
	for (; vert_it != vert_it_end ; ++vert_it ) 
    {
        if ( !(*this)[*vert_it].alive )
            continue;
    	tie(v_it, v_it_end) = vertices(*this);
    	for (; v_it != v_it_end ; ) 
        {
            if ( !(*this)[*v_it].alive ) {
                ++v_it;
                continue;
            }
            if ( edge(*v_it, *vert_it, *this).second ) {
                out_file << "1";
            } else {
                out_file << "0";
            }
            ++v_it;
            if (v_it == v_it_end )
                out_file << std::endl;
            else
                out_file << ", ";
    	}
    }
    out_file << std::endl;
    
    
    
	out_file.close();     
}

void sg_graph::print_graph_to_file(const char* filename, int l_id, bool non_mst_edges) {
	ofstream out_file;
	sg_o_edge_it 	out_edge_it, out_edge_it_end;
	sg_adj_it 	adj_it, adj_it_end;
	
	// Prepare the file into which to write the graph
	out_file.open(filename);//formerly out_file
	out_file << "graph: { layoutalgorithm: forcedir display_edge_labels: yes \n";
	
	// Traverse vertices
	sg_vertex_d	vert_desc;
	sg_vertex_it	vert_it, vert_it_end;

	tie(vert_it, vert_it_end) = vertices(*this);
	for (; vert_it != vert_it_end ; ++vert_it ) {
		tie(out_edge_it, out_edge_it_end)  = out_edges(*vert_it, *this);
		tie(adj_it, adj_it_end) 		   = adjacent_vertices(*vert_it, *this);
		vert_desc = vertex(*vert_it, *this);//get the vertex descriptor
		out_file << "node: { title: \"" << vert_desc << "\"  label: \" v" 
				 <<  vert_desc << " w=" << (*this)[vert_desc].w << " s=" 
				 << (*this)[vert_desc].s << "\" } \n";
	}

	// Traverse Edges
	sg_edge_it tree_edge_it,tree_edge_it_end;
	sg_edge_d ed;
	tie( tree_edge_it, tree_edge_it_end ) = edges( *this );
	int counted_edge = 0;
	for (; tree_edge_it != tree_edge_it_end; tree_edge_it++) {
			
			counted_edge++;
			ed = *tree_edge_it;
			if ( !(non_mst_edges == false && (*this)[ed].is_in_mst == false) ) {
				out_file << "edge: { source: \"" << source(ed,*this);
				out_file	<< "\" target: \"" <<  target(ed,*this);
				out_file	<< "\" label: \"w=" << (*this)[ed].w;
				out_file << " l" << l_id << "0=" << (*this)[ed].label[l_id][0];
				out_file << " l" << l_id << "1=" << (*this)[ed].label[l_id][1];
				out_file << " mst=" << (*this)[ed].is_in_mst;
				out_file << " V20=" << (*this)[ed].is_in_V_2_perm[0];
				out_file << " V21=" << (*this)[ed].is_in_V_2_perm[1];
				out_file	<<"\" } \n";
			}
	}

	out_file << "}";
	out_file.close();     
}	                                                        

void sg_graph::copy_edge_values( sg_edge_d ed1, sg_edge_d ed2 ) {
    (*this)[ed1].w = (*this)[ed2].w;
	(*this)[ed1].w_inv = (*this)[ed2].w_inv;
	for( int i = 0 ; i < 3 ; ++i ) {
		(*this)[ed1].label[i][0] = (*this)[ed2].label[i][0]; 
		(*this)[ed1].label[i][1] = (*this)[ed2].label[i][1]; 
	}
	(*this)[ed1].is_in_pipe = (*this)[ed2].is_in_pipe;
	(*this)[ed1].is_in_V_2_perm[0] = (*this)[ed2].is_in_V_2_perm[0];
	(*this)[ed1].is_in_V_2_perm[1] = (*this)[ed2].is_in_V_2_perm[1];
	(*this)[ed1].is_in_V_2 = (*this)[ed2].is_in_V_2;
	(*this)[ed1].is_in_mst = (*this)[ed2].is_in_mst;
	(*this)[ed1].x = (*this)[ed2].x;
	(*this)[ed1].y = (*this)[ed2].y;        
}

sg_vertex_d sg_graph::get_first( sg_edge_d ed) {
	sg_vertex_d vd, vd2;
	vd = source( ed, (*this) );
	vd2 = target( ed, (*this) );
	if ( int(vd) < int(vd2) )
		return vd;
	else
		return vd2;
}

sg_vertex_d sg_graph::get_second( sg_edge_d ed) {
	sg_vertex_d vd, vd2;
	vd = source( ed, (*this) );
	vd2 = target( ed, (*this) );
	if ( int(vd) < int(vd2) )
		return vd2;
	else
		return vd;
}

int sg_graph::number_of_nonmst_edges( int& cost ) {
	sg_edge_it edge_it, edge_it_end;
	tie(edge_it,edge_it_end) = edges(*this);
	int count=0;
	cost = 0;
	for (; edge_it != edge_it_end; ++edge_it) {
		if ( (*this)[*edge_it].is_in_mst == false ) {
			cout << " NON MST EDGE " << source(*edge_it, (*this)) << ":" 
			     << target(*edge_it, (*this)) 
			     << " w=" << (*this)[*edge_it].w << endl;
			++count;
			cost += (*this)[*edge_it].w;
		}
	}
	return count;
}

/* andreas
 * 
 * Needed to adhere to the convention that [0] is the label from source
 * to target and [1] from target to source. This function returns the
 * label corresponding to the direction from vd across ed to the other
 * vertex if outbound == 1. Otherwise it returns the inbound label
 * from the other vertex across ed into vd.
 */
int sg_graph::get_label( sg_vertex_d vd, sg_edge_d ed, int l_id, bool outbound) {   
	if ( vd == this->get_first(ed) )
		return (*this)[ed].label[l_id][int(!outbound)]; // if vd first, then 0 is the outbound label
	else if ( vd == this->get_second(ed) )
		return (*this)[ed].label[l_id][int(outbound)]; // vice versa to above
	return -2;
} 

// coming from vd1 into vd2
int sg_graph::get_sweep_cost( sg_vertex_d vd1, sg_vertex_d vd2 ) {
	
	int dir_ind = get_sweep_dir_index( vd1, vd2 );
	return (*this)[vd2].ww[dir_ind];
}

int sg_graph::get_sweep_dir_index( sg_vertex_d vd1, sg_vertex_d vd2) {
	sg_edge_d ed;
	sg_o_edge_it out_edge_it, out_edge_it_end;
	tie(out_edge_it, out_edge_it_end) = out_edges( vd2, *this);
	int dir_ind = 0;
	for (;out_edge_it != out_edge_it_end; out_edge_it++) {
		ed = *out_edge_it;
		if ( vd1 == source( ed, *this ) || vd1 == target( ed, *this ) )
			break;
		++dir_ind;
	}
	if ( dir_ind >= SG_GRAPH_MAX_VERTEX_DEGREE ) 
		cout << "ERROR G_GRAPH_MAX_VERTEX_DEGREE exceeded" << endl;
	return dir_ind;
}


sg_vertex_d sg_graph::get_neighbor( sg_vertex_d vd, sg_edge_d e) {
  if ( vd == source( e, *this ) )
    return target( e, *this );
  else
    return source( e, *this );  
}

float sg_graph::distance_to_edge( sg_vertex_d vd1, sg_vertex_d vd2, int x, int y ) {
  sg_o_edge_it	out_edge_it, out_edge_it_end;
  sg_edge_d shared_edge;
	
  cout << " ------ distance to edge " << vd1 << ":" << vd2 << " from " << x << ":" << y << endl;
	
	tie(out_edge_it, out_edge_it_end) = out_edges(vd1, *this);
  bool shared_edge_found = false;
	for( ; out_edge_it != out_edge_it_end ; out_edge_it++ ) {
    if ( source(*out_edge_it, *this) == vd2 || target(*out_edge_it, *this) == vd2 ) {
      shared_edge = *out_edge_it;
      shared_edge_found = true;
      break;
    }
  }
  
  if ( shared_edge_found == false )
    return 10000.0;
	
  cout << " -------- edge is at " << (*this)[shared_edge].x << ":" << (*this)[shared_edge].y << endl;
	
  float distance =  ( (*this)[shared_edge].x - x ) * ( (*this)[shared_edge].x - x ) + ( (*this)[shared_edge].y - y ) * ( (*this)[shared_edge].y - y );
  distance = sqrt( distance );
  return distance;
}

float sg_graph::distance_between( sg_vertex_d v, sg_vertex_d w ) {
    float distance =  pow( (*this)[v].x - (*this)[w].x, 2 ) 
           + pow( (*this)[v].y - (*this)[w].y , 2 ) ;
    distance = sqrt( distance );
    return distance;
}

float sg_graph::distance_between( sg_vertex_d v, sg_edge_d e ) {
    float distance =  pow( (*this)[v].x - (*this)[e].x, 2 ) 
           + pow( (*this)[v].y - (*this)[e].y, 2 ) ;
    distance = sqrt( distance );
    return distance;
}

float sg_graph::distance_between( sg_edge_d ee, sg_edge_d e ) {
    float distance =  pow( (*this)[ee].x - (*this)[e].x, 2 ) 
           + pow( (*this)[ee].y - (*this)[e].y , 2 ) ;
    distance = sqrt( distance );
    return distance;
}

#include "sg_label_computer.h"

sg_label_computer::sg_label_computer() {
	max_traversal = 2000;
}

void sg_label_computer::compute_labels( sg_graph& SG, int l_id ) {
	SG.label_id = l_id;
	cout << "Label Computer: computing labels " << endl;
	if ( DEBUG_LBL >= 1 ) cout << " Cleaning up graph" << endl;
	clean_up_graph( SG );
	if ( DEBUG_LBL >= 1 ) cout << " Convert to tree (MST)" << endl;
	convert_graph_to_tree( SG );
	if ( DEBUG_LBL >= 1 ) cout << " Compute save sweeps" << endl;
	compute_save_sweep( SG );
	//SG.print_graph_to_file("/Users/andreas/graph.gdl", 2, 0);
	if ( DEBUG_LBL >= 1 ) cout << " Initial fill of pipeline with leaf edges" << endl;
	leaf_edge_pipeline_fill( SG );
	if ( DEBUG_LBL >= 1 ) cout << " Traverse pipeline" << endl;
	pipeline_traverse( SG, l_id );
}

//******* Main process *******(fold)

void sg_label_computer::clean_up_graph(sg_graph& SG) {
	int i, w_max = 0;
	sg_edge_it e_i, e_i_end;
	tie( e_i , e_i_end ) = edges(SG);
	for ( ; e_i != e_i_end ; ++e_i ) {
		if ( SG[*e_i].w > w_max )
			w_max = SG[*e_i].w;
	}
	++w_max;
	tie( e_i , e_i_end ) = edges(SG);
	for ( ; e_i != e_i_end ; ++e_i ) {
		SG[*e_i].w_inv = w_max - SG[*e_i].w;
		SG[*e_i].is_in_pipe = false;
		SG[*e_i].is_in_V_2 = false;
		SG[*e_i].is_in_mst = false;
		for ( i = 0 ; i < NUMBER_OF_LABELS ; ++i ) {
			SG[*e_i].label[ i ][0] = -1;
			SG[*e_i].label[ i ][1] = -1;
		}
	}
}

void sg_label_computer::convert_graph_to_tree(sg_graph& SG) {
	MST_edges.clear();
	kruskal_minimum_spanning_tree(SG, std::back_inserter(MST_edges),
		boost::weight_map(get(&sg_edge::w_inv, SG)));
	// Set all edges of the MST to valid edges for the strategy
	vector<sg_edge_d>::iterator i, i_end;
	i = MST_edges.begin();
	i_end = MST_edges.end();
	if ( DEBUG_LBL >= 2 )
		cout << "     Showing MST edges:" << endl;
	for ( ; i != i_end ; ++i ) {
		if ( DEBUG_LBL >= 2 )
			cout << "    " << *i ;
		SG[*i].is_in_mst = true;
	}
	if ( DEBUG_LBL >= 2 )
		cout << endl;
	
}

void sg_label_computer::compute_save_sweep(sg_graph& SG) {
	int total_max_s = 0;
	int local_s = 0;
	sg_vertex_d		vd;
	sg_vertex_it	vert_it, vert_it_end;
	sg_o_edge_it	out_edge_it, out_edge_it_end;

	tie(vert_it, vert_it_end) = vertices(SG);
	for (; vert_it != vert_it_end; ++vert_it) {
		tie(out_edge_it, out_edge_it_end) = out_edges(*vert_it, SG);
		vd = *vert_it;
		local_s = SG[vd].w;
		for (;out_edge_it != out_edge_it_end; out_edge_it++) {
			if (SG[*out_edge_it].is_in_mst == true) {
				local_s += SG[*out_edge_it].w;
			}
		}
		SG[vd].s = local_s;
		if (local_s > total_max_s)
			total_max_s = local_s;
	}
}

void sg_label_computer::leaf_edge_pipeline_fill(sg_graph& SG) {
	sg_vertex_it		vert_it, vert_it_end;
	sg_o_edge_it	out_edge_it, out_edge_it_end;
	tie(vert_it, vert_it_end) = vertices(SG);
	if (DEBUG_PTRAV >= 1) 
		cout << " Checking " << num_vertices(SG) << " vertices" << endl;
	if (DEBUG_PTRAV >= 2) 
		cout << "	 Filling pipeline with: ";
	
	for (; vert_it != vert_it_end; ++vert_it) {
		SG[*vert_it].mst_degree = get_mst_degree( *vert_it, SG );
		if (SG[*vert_it].mst_degree == 1) {
			tie(out_edge_it, out_edge_it_end)	 = out_edges(*vert_it, SG);
			for ( ; out_edge_it != out_edge_it_end ; ++out_edge_it) {
				if ( SG[*out_edge_it].is_in_mst == true ) {
					if (DEBUG_PTRAV >= 2) 
					cout << *out_edge_it << " ";
					edge_pipeline.push_back(*out_edge_it); 
					SG[*out_edge_it].is_in_pipe = true;
				}
			}
		}
	}
	if (DEBUG_PTRAV >= 2) 
		cout << endl;
}

void sg_label_computer::pipeline_traverse(sg_graph& SG, int l_id) {
	sg_edge_d ed;
	int valid = 0, traversal = 0;
	while (edge_pipeline.size() && traversal <= max_traversal ) {
		++traversal;
		if ( DEBUG_PTRAV >= 2 ) 
			cout << "	 " << traversal <<" Pipe Traversal " << endl;
		ed = *(edge_pipeline.begin());
		SG[ed].is_in_pipe = false;
		edge_pipeline.pop_front();
		valid = edge_check_validity(ed, l_id, SG);
		if ( valid % 2 == 1 ) {
			compute_label(ed, 0, l_id, SG);
			valid -= 1;
		}
		if ( valid == 2 )
			compute_label(ed, 1, l_id, SG);
	}
}

//*** (end)

//******* Computing labels *******

/* 
 * An edge can be ready for assignment if either the source or sink
 * has all outgoing edges assigned with a label The return_value can
 * either be 0 = no edge is valid, 1 = source label can be computed,
 * 2 = target label can be computer or 3 = bot labels can be computed
 * the source is not source(), but the smaller vertex_descriptor Since
 * edges (1,2) (2,1) can both appear we need a another convention.
 */

int sg_label_computer::edge_check_validity( sg_edge_d ed, int l_id, sg_graph& SG ) {

	bool valid = true;
	int return_value = 0;
	sg_vertex_d vd;
	sg_o_edge_it out_edge_it, out_edge_it_end;

	// Checking for label in direction 0
	// from first to second, hence check outgoing labels for second
	if ( DEBUG_PTRAV >= 4 )
		cout << "		 Checking validity " << ed << " from first to second (dir 0)" << endl;
	vd = get_second(ed, SG);
	// Check around vd whether all other outgoing labels are computed
	sg_o_edge_it oe_i, oe_i_end;
	tie(oe_i, oe_i_end) = out_edges(vd, SG);
	for (valid = true; oe_i != oe_i_end; oe_i++ ) {
		if ( *oe_i != ed && SG[*oe_i].is_in_mst == true) {
			if ( !out_going_label_computed(vd, *oe_i, l_id, SG) )
				valid = false;
		}
	}
	if ( valid == true )
		return_value += 1;
		if ( DEBUG_PTRAV >= 4 && valid == true )
		cout << "		 ++-> Edge " << ed << " valid assig. towards " << vd << endl;


	// Check for the other direction
	if ( DEBUG_PTRAV >= 4 )
		cout << "		 Checking validity " << ed << " from second to first (dir 1)" << endl;	
	vd = get_first(ed, SG);
	tie(oe_i, oe_i_end) = out_edges(vd, SG);
	for (valid = true; oe_i != oe_i_end && valid == true ; oe_i++ ) {
		if ( *oe_i != ed && SG[*oe_i].is_in_mst == true ) {
			if ( !out_going_label_computed(vd, *oe_i, l_id, SG) )
				valid = false;
		}
	}
	if ( valid == true)
		return_value += 2;
	if ( DEBUG_PTRAV >= 4 && valid == true )
		cout << "		 ++-> Edge " << ed << " valid assig. towards " << vd << endl;
			
	return return_value;
}

bool sg_label_computer::out_going_label_computed( sg_vertex_d vd, sg_edge_d ed, int l_id, sg_graph& SG) {
	short direction = 0;
	direction = ( vd == get_first( ed, SG ) ) ? 0 : 1;
	
	if ( DEBUG_PTRAV >= 5 )
		cout << "			Is the outgoing label of " << ed << " for " << vd << " computed?" << endl;
	
	if ( DEBUG_PTRAV >= 5 )
		cout << "			--> use dir=" << direction << " labels are: 0) " << SG[ed].label[l_id][0] << " and 1) " << SG[ed].label[l_id][1] << endl;

	if ( DEBUG_PTRAV >= 5 && SG[ed].label[l_id][direction] > 0 )
		cout << "			-> " << vd << " " << ed << ": out label is computed? YES! " << endl;
	if ( DEBUG_PTRAV >= 5 && SG[ed].label[l_id][direction] <= 0 )
		cout << "			-> NO! outlabel is not computed " << endl;
	if (SG[ed].label[l_id][direction] > 0)
		return true;
	else
		return false;		
}

/* andreas
 * 
 * Major part. Here we compute the actual value of the label for edge ed.
 * Into direction di
 */
void sg_label_computer::compute_label( sg_edge_d ed, int di, int l_id, sg_graph& SG ) {
	
	if ( DEBUG_LBL >= 4 )
		cout << "		 +++ Computing the label on " << ed << " direction " << di << " label_type " << l_id << endl;
	
	if ( SG[ed].label[l_id][di] > 0 ) {
		//cout << "		 --- Abort - already computed" << endl;
		return;	 
	}
	sg_vertex_d vd;
	int max_cost = 0;
	
	// To compute direction 0, i.e. label from first to second
	// we need to look at the second vertex and its neighbors
	// For direction 1 the converse
	vd = ( di == 0 ) ? get_second( ed, SG ) : get_first( ed, SG );
	
	if ( l_id == 4 ) {
		// need to compute .s for this direction
		// figure out which edge ed is of vd
		sg_o_edge_it out_edge_it, out_edge_it_end;
		tie(out_edge_it, out_edge_it_end) = out_edges( vd, SG);
		int n_edges = 0;
		//cout << " Looking for " << target(*out_edge_it,SG) << ":" << source(*out_edge_it,SG) << endl;
		//cout << " at VVV=" << vd << endl;
		for (;out_edge_it != out_edge_it_end; out_edge_it++) {
			//cout << " Checking vertex weight " << target(*out_edge_it,SG) << ":" << source(*out_edge_it,SG) << endl;
			if ( *out_edge_it == ed )
				break;
			++n_edges;
		}
		//cout << "found index" << n_edges << endl;
		SG[vd].s = SG[vd].ww[n_edges];
	}
	
	int neighbor_size = SG[vd].mst_degree - 1;
	if ( neighbor_size >= 1 )	 { // NOT A LEAF
		if ( DEBUG_LBL >= 5 )
			cout << "			Not a leaf " << endl;
		edge_vector e_nei( neighbor_size );
		edge_vector::iterator n_start=e_nei.begin(), n_end;
		// fill e_nei with all edge_descriptors and rho resp. to the label
		int n_neigh = collect_neighbors( vd, ed, e_nei, l_id, SG);
		// sort e_nei, the vector of edge neighbors
		n_end = e_nei.end();
		sort( n_start, n_end, edge_comparison);
		// find the maximum cost, the labels are implicitly given in e_nei
		// e_nei.second is label - edge_weight
		int max_i = 0, max_label = 0;
		max_cost = find_max_cost(max_i, max_label, e_nei, n_neigh, SG);
		
		// *** For hybrid strategies
		// compute the batch index k at the maximum: a - rho_{max_i}
		int max_k = max_label - e_nei[max_i].second;
		//partition if necessary and then update the maximum cost
		int penalty_w = SG[ed].w;
		if ( l_id == 1 && n_neigh > 1)
			partition_hybrid_h( e_nei, n_neigh, penalty_w, max_k, max_i, max_label, SG);
		else if ( l_id == 2 && n_neigh > 1)
			partition_hybrid_t( e_nei, vd, n_neigh, penalty_w, max_cost, SG );
		if ( (l_id == 1 || l_id == 2) && n_neigh > 1)
			max_cost = find_max_cost_hybrid( max_i, e_nei, penalty_w, n_neigh, SG );
	}	 
	if ( DEBUG_LBL >= 5 )
		cout << "			Max_cost " << max_cost << " save_sweep" << SG[vd].s << endl;
	
	// assign label value - if neighbor is leaf automatically gets SG[vd].s
	SG[ed].label[l_id][di] = max( max_cost, SG[vd].s );
	
	// PIPELINE FILL: Check whether to add the neighbor to compute next label (fold)
	if ( DEBUG_LBL >= 5 )
		cout << "			Refill pipeline " << endl;	
	vd = ( di == 1 ) ? get_second( ed, SG ) : get_first( ed, SG );
	sg_o_edge_it oe_i, oe_i_end;
	tie(oe_i, oe_i_end) = out_edges(vd, SG);
	for ( ; oe_i != oe_i_end; ++oe_i ) {
		if ( *oe_i != ed && SG[*oe_i].is_in_mst == true ) {
			// check whether it does not already have label into vd computed
			// get_label with 0 gets inbound label across ed into vd
			if ( SG.get_label( vd, *oe_i, l_id, 0) <= 0 && SG[*oe_i].is_in_pipe == false) {
				edge_pipeline.push_back(*oe_i);
				SG[*oe_i].is_in_pipe = true;
			}
		}
	}				
	// (end)
}

int sg_label_computer::find_max_cost( int& max_i, int& max_l, edge_vector& n_e,
                                      int n_e_size, sg_graph& SG ) {
	if ( DEBUG_PTRAV >= 4) {
		cout << "		 -> Finding max cost for " << n_e_size << " neighbors. "
		     << endl;
	}
	int block_cost = 0, cost = 0, max_cost = 0, label = 0;
	max_l = 0;

	// WATCH OUT: the computation is the same for contiguous and non-cont
	// but the order here is for non-cont, i.e. the first vertex here
	// is also the first vertex to be cleared in the non-con strategy
	// but for contigous strategies when i = 0 the last vertex cost is computed
	for ( int i = 0 ; i < n_e_size ; i++ ) {
		label = n_e[i].second + SG[ n_e[i].first ].w;
		cost = block_cost + label;
		if ( DEBUG_LBL >= 4)
			cout << "			" << n_e[i].first << ": rho=" << n_e[i].second << " lab=" << label << " cost=" << cost << endl;
		if ( cost > max_cost) {
			max_i = i;
			max_cost = cost;
		}
		if ( label > max_l )
			max_l = label;
		block_cost += SG[ n_e[i].first ].w;
	}
	return max_cost;
}

int sg_label_computer::find_max_cost_hybrid( int& max_i, edge_vector& n_e, int penalty_w, int n_n_e, sg_graph& SG) {
	if (DEBUG_LBL >= 5)
		cout << "		 -> Finding hybrid max cost hybrid for " << n_n_e << " neighbors " << endl;
	sg_edge_d ed;
	int all_max_cost = 0, is_in_V_2 = 0, label, cost = 0;
	int block_cost[2];
	int max_cost[2];
	max_cost[0] = 0; max_cost[1] = 0;
	block_cost[0] = 0; block_cost[1] = 0;

	for (int i = 0; i < n_n_e; i++) {
		ed = n_e[i].first;
		label = n_e[i].second + SG[ed].w;
		is_in_V_2 = SG[ed].is_in_V_2; // either 0 or 1
		cost = block_cost[is_in_V_2] + label;
		if ( DEBUG_LBL >= 5)
			cout << "			" << n_e[i].first << ": rho=" << n_e[i].second << " lab=" << label << " cost=" << cost << endl;
		if ( cost > max_cost[is_in_V_2] ) {
			max_i = i;
			max_cost[is_in_V_2] = cost;
		}
		block_cost[is_in_V_2] += SG[ed].w;
	}
	//the smaller one gets the penalty weight
	//(should be V_1 since this is how the current partitioning assigns it)
	//but you can never be sure! So we are checking it.
	if (max_cost[0] > max_cost[1])
		all_max_cost = max(max_cost[1] + penalty_w, max_cost[0]);
	else
		all_max_cost = max(max_cost[0] + penalty_w, max_cost[1]);
	return all_max_cost;
}

//******* Hybrid Partitioning ******* (fold)

int sg_label_computer::partition_hybrid_h(edge_vector& neighbors, int n_neigh, int penalty, int max_k, int max_i, int max_lambda, sg_graph& SG) {
		int i = 0, j = 0;
	int n_vertices = n_neigh + 2;
	if (max_i < 1)
		return 0;
	if (n_neigh < 2)
		return 0;

	if (DEBUG_LBL >= 1)
		cout << "---------> Partitioning into V_1 and V_2" << endl;

	//--- sum up the weights into w_sum
	if (DEBUG_LBL >= 2)
		cout << "Penalty weight: " << penalty << endl;

	if (DEBUG_LBL >= 2)
		cout << "Summing up the total 'relevant' edge weight: ";

	int w_sum = 0;
	vector<int> w_sum_v(max_i + 1);
	int counter = 1;
	//w_sum_v[1] contains the edge weight of edge max_i (v_m)
	//w_sum_v[2] contains the edge weight of edge max_i (v_m) and max_i-1 (v_{m-1})
	for(i = max_i; i >= 0; i--) {//summing up only to the maximal cost
		w_sum += SG[neighbors[i].first].w;
		w_sum_v[counter] = w_sum;
		counter++;
	}
	if (DEBUG_LBL >= 2)
		cout << " =	 " << w_sum << endl;

	//---------------------------------------
	//--- do we have to partition at all?
	//---------------------------------------
	int z = ceil(w_sum/2 - penalty/2);
	if(z < 1) {
		//partition is useless, set V_2 = emptyset
		if (DEBUG_LBL >= 2)
			cout << "	 !!! --- !!! Setting V_2 to empty-set " << endl;
		for(i = 0; i < n_neigh; i++){
			SG[neighbors[i].first].is_in_V_2 = 0;
		}
		return 0;
	}

	vector<int> dummy_v(w_sum + 2); //one for the zero row, then 1 to w_sum+1
	vector< vector<int> >* A = new vector< vector<int> >(n_vertices,dummy_v);
	vector< vector<int> >* K_1 = new vector< vector<int> >(n_vertices,dummy_v);
	vector< vector<int> >* K_2 = new vector< vector<int> >(n_vertices,dummy_v);
	vector< vector<int> >* S_1 = new vector< vector<int> >(n_vertices,dummy_v);
	vector< vector<int> >* S_2 = new vector< vector<int> >(n_vertices,dummy_v);
	vector< vector<int> >* H = new vector< vector<int> >(n_vertices,dummy_v);
	vector< vector<int> >* PATH = new vector< vector<int> >(n_vertices,dummy_v);

	//now the big chunk
	//be aware of the indexing issue!!!

	int c_i = 0, lambda_i = 0;
	bool i_into_V_1 = false;
	int rho_i, k_i, current_total_w;
	sg_edge_d c_n; //to hold the current neighbor considered

	for (i = 0; i <= max_i + 1; i++) {
		for (j = 0; j <= w_sum; j++) {
			if (DEBUG_LBL >= 3) cout << " At " << i << " " << j << " : ";
			if (j == 0 || i == 0) { // set initial values
				//we need particular starting values due to the fact that also populat V_1
				if (DEBUG_LBL >= 3) cout << "Setting initial value" << endl;
				(*PATH)[i][j] = 2;
				(*A)[i][j] = 0;
				(*K_2)[i][j] = 0;
				(*S_2)[i][j] = 0;
				if (i != 0) {
					c_n = neighbors[max_i - i + 1].first;
					c_i = SG[c_n].w;
					lambda_i	= neighbors[max_i - i + 1].second - SG[c_n].w;
					rho_i			= neighbors[max_i - i + 1].second;
					k_i			= max_lambda - rho_i;
					if ( k_i < (*K_1)[i-1][j] - (*S_1)[i-1][j] || (*K_1)[i-1][j] == 0) {
						if (DEBUG_LBL >= 3) cout << " V_1 new max" << endl;
						(*K_1)[i][j] = k_i;
						(*S_1)[i][j] = c_i;
					}
					else {
						if (DEBUG_LBL >= 3) cout << " V_1 max kept" << endl;
						(*K_1)[i][j] = (*K_1)[i-1][j];
						(*S_1)[i][j] = (*S_1)[i-1][j] + c_i;
					}
				}
				else {
					(*K_1)[i][j] = 0;
					(*S_1)[i][j] = 0;
				}
			}
			else {
				c_n = neighbors[max_i - i + 1].first;
				c_i = SG[c_n].w;
				lambda_i	= neighbors[max_i - i + 1].second - SG[c_n].w;
				rho_i			= neighbors[max_i - i + 1].second;
				k_i			= max_lambda - rho_i;
				if (DEBUG_LBL >= 3) cout << "Check for fit for " << c_i << " --- ";
				if (c_i > j) {
					if (DEBUG_LBL >= 3) cout << " c_i > j : using previous row i-1" << endl;
					//use previous entry, i-1 does not fit into j-th column, i.e. not into V_2
					(*PATH)[i][j] = 0;
					(*A)[i][j] = (*A)[i-1][j];
					if ( k_i < (*K_1)[i-1][j] - (*S_1)[i-1][j] || (*K_1)[i-1][j] == 0) {
						(*K_1)[i][j] = k_i;
						(*S_1)[i][j] = c_i;
					}
					else {
						(*K_1)[i][j] = (*K_1)[i-1][j];
						(*S_1)[i][j] = (*S_1)[i-1][j] + c_i;
					}
					(*K_2)[i][j] = (*K_2)[i-1][j];
					(*S_2)[i][j] = (*S_2)[i-1][j];
				}
				else { //i could fit
					if (DEBUG_LBL >= 3) cout << " c_i <= j : i can fit --- ";
					if( (*A)[i-1][j-c_i] + c_i >= (*A)[i-1][j] ) { //does it fit better or equally well?
						(*A)[i][j] = (*A)[i-1][j-c_i] + c_i;	//new maximum achievable weight
						//check whether we lose the position of the max in V_2
						if (DEBUG_LBL >= 3) cout << " better fit --- ";
						if ( k_i < ((*K_2)[i-1][j-c_i] - (*S_2)[i-1][j-c_i]) || (*K_2)[i-1][j-c_i] == 0) {
							if (DEBUG_LBL >= 3) cout << " V_2 new max --- ";
							(*K_2)[i][j] = k_i;
							(*S_2)[i][j] = c_i;
						}
						else {
							if (DEBUG_LBL >= 3) cout << " V_2 max kept --- ";
							(*K_2)[i][j] = (*K_2)[i-1][j-c_i];
							(*S_2)[i][j] = (*S_2)[i-1][j-c_i] + c_i;
						}
						(*K_1)[i][j] = (*K_1)[i-1][j-c_i];
						(*S_1)[i][j] = (*S_1)[i-1][j-c_i];
						(*PATH)[i][j] = 1;
						if ((*A)[i][j] == (*A)[i-1][j] ) {
							//we could also choose the upper row entry ignoring element i for V_2
							//COMPARE PARTITIONS
							int alt_H;
							if (DEBUG_LBL >= 3) cout << " Compare partitions --- ";
							current_total_w = w_sum_v[i];//total edge weight of all edges up to i
							(*H)[i][j] =
									(*K_1)[i][j] + (*K_2)[i][j]
								+ current_total_w - (*S_1)[i][j] - (*S_2)[i][j]
								- abs( (*S_1)[i][j] - (*S_2)[i][j] - (*K_1)[i][j] + (*K_2)[i][j] - penalty);
							//compute alternative H if we take i into V_1
							if ( k_i < (*K_1)[i-1][j] - (*S_1)[i-1][j] || (*K_1)[i-1][j] == 0) {
								if (DEBUG_LBL >= 4) cout << " compute alt_H with V_1 new max" << endl;
								alt_H =
										k_i + (*K_2)[i-1][j]
									+ current_total_w - c_i - (*S_2)[i-1][j]
									- abs( c_i - (*S_2)[i-1][j] - k_i + (*K_2)[i-1][j] - penalty);
							}
							else {
								if (DEBUG_LBL >= 4) cout << " compute alt_H with V_1 max kept" << endl;
								alt_H =
										(*K_1)[i-1][j] + (*K_2)[i-1][j]
									+ current_total_w - (*S_1)[i-1][j] - (*S_2)[i-1][j] - c_i
									- abs(
												(*S_1)[i-1][j] + c_i - (*S_2)[i-1][j]
											- (*K_1)[i-1][j] + (*K_2)[i-1][j]
											- penalty);
							}
							if (alt_H > (*H)[i][j])
								i_into_V_1 = true;
							else
								i_into_V_1 = false;
							if ( i_into_V_1 == true) {
								if (DEBUG_LBL >= 3) cout << " V_1 better choice --- ";
								(*H)[i][j] = alt_H;
								(*PATH)[i][j] = 0;
								if ( k_i < (*K_1)[i-1][j] - (*S_1)[i-1][j] || (*K_1)[i-1][j] == 0) {
									if (DEBUG_LBL >= 3) cout << " V_1 new max" << endl;
									(*K_1)[i][j] = k_i;
									(*S_1)[i][j] = c_i;
								}
								else {
									if (DEBUG_LBL >= 3) cout << " V_1 max kept" << endl;
									(*K_1)[i][j] = (*K_1)[i-1][j];
									(*S_1)[i][j] = (*S_1)[i-1][j] + c_i;
								}
								(*K_2)[i][j] = (*K_2)[i-1][j];
								(*S_2)[i][j] = (*S_2)[i-1][j];
							}
							else {
								if (DEBUG_LBL >= 3) cout << " Stick with V_2" << endl;
							}
						}
						else {
							if (DEBUG_LBL >= 3) cout << " previous row worse, no comparison" << endl;
						}
					}
					else { //previous row is better
						(*PATH)[i][j] = 0;
						if (DEBUG_LBL >= 3) cout << " worse fit using previous row --- " << endl;
						(*A)[i][j] = (*A)[i-1][j];
						if ( k_i < (*K_1)[i-1][j] - (*S_1)[i-1][j] || (*K_1)[i-1][j] == 0) {
							if (DEBUG_LBL >= 3) cout << " V_1 new max" << endl;
							(*K_1)[i][j] = k_i;
							(*S_1)[i][j] = c_i;
						}
						else {
							if (DEBUG_LBL >= 3) cout << " V_1 max kept" << endl;
							(*K_1)[i][j] = (*K_1)[i-1][j];
							(*S_1)[i][j] = (*S_1)[i-1][j] + c_i;
						}
						(*K_2)[i][j] = (*K_2)[i-1][j];
						(*S_2)[i][j] = (*S_2)[i-1][j];
					}
				}
			}
		}
	}
	int j_start_print = 0;
	int i_start_print = 0;
	if (DEBUG_LBL >= 3) {
		cout << endl << "A" << endl;
		for (i = i_start_print; i <= max_i+1; i++) {
			cout << SG[neighbors[max_i - i + 1].first].w << " (" << w_sum_v[i] << "): ";
			for (j = j_start_print; j <= w_sum ; j++) {
				cout << (*A)[i][j] << "	 ";
			}
			cout << ";" << endl;
		}
		cout << endl << "K_1" << endl;
		for (i = i_start_print; i <= max_i+1; i++) {
			cout << SG[neighbors[max_i - i + 1].first].w << " (" << w_sum_v[i] << "): ";
			for (j = j_start_print; j <= w_sum ; j++) {
				cout << (*K_1)[i][j] << "	 ";
			}
			cout << ";" << endl;
		}
		cout << endl << "K_2" << endl;
		for (i = i_start_print; i <= max_i+1; i++) {
			cout << SG[neighbors[max_i - i + 1].first].w << " (" << w_sum_v[i] << "): ";
			for (j = j_start_print; j <= w_sum ; j++) {
				cout << (*K_2)[i][j] << "	 ";
			}
			cout << ";" << endl;
		}
		cout << endl << "S_1" << endl;
		for (i = i_start_print; i <= max_i+1; i++) {
			cout << SG[neighbors[max_i - i + 1].first].w << " (" << w_sum_v[i] << "): ";
			for (j = j_start_print; j <= w_sum ; j++) {
				cout << (*S_1)[i][j] << "	 ";
			}
			cout << ";" << endl;
		}
		cout << endl << "S_2" << endl;
		for (i = i_start_print; i <= max_i+1; i++) {
			cout << SG[neighbors[max_i - i + 1].first].w << " (" << w_sum_v[i] << "): ";
			for (j = j_start_print; j <= w_sum ; j++) {
				cout << (*S_2)[i][j] << "	 ";
			}
			cout << ";" << endl;
		}
	}
	if (DEBUG_LBL >= 3) cout << endl << "H" << endl;
	for (i = i_start_print; i <= max_i+1; i++) {
		if (DEBUG_LBL >= 3) cout << SG[neighbors[max_i - i + 1].first].w << " (" << w_sum_v[i] << "): ";
		current_total_w = w_sum_v[i];//total edge weight of all edges up to i
		for (j = j_start_print; j <= w_sum ; j++) {
			(*H)[i][j] =
					(*K_1)[i][j] + (*K_2)[i][j]
				+ current_total_w - (*S_1)[i][j] - (*S_2)[i][j]
				- abs( (*S_1)[i][j] - (*S_2)[i][j] - (*K_1)[i][j] + (*K_2)[i][j] - penalty);
			if (DEBUG_LBL >= 3) cout << (*H)[i][j] << "	 ";
		}
		if (DEBUG_LBL >= 3) cout << ";" << endl;
	}
	if (DEBUG_LBL >= 3) {
		cout << endl << "PATH" << endl;
		for (i = i_start_print; i <= max_i+1; i++) {
			cout << SG[neighbors[max_i - i + 1].first].w << " (" << w_sum_v[i] << "): ";
			for (j = j_start_print; j <= w_sum ; j++) {
				cout << (*PATH)[i][j] << "	";
			}
			cout << ";" << endl;
		}
	}

	//go through the last row and find the optimal partition
	i = max_i + 1; // the last row
	int m_h = 0, m_j = 0;
	for (j = 0; j <= w_sum; j++) {
		if ( (*H)[i][j] > m_h) {
			m_h = (*H)[i][j];//killer line for another wasted day
			m_j = j;
		}
	}
	// construct the partition leading to i, m_j
	for (i = max_i + 1; i > 0; i--) {
		if ( (*PATH)[i][m_j] == 0) {//jump to prev. row
			if (DEBUG_LBL >= 3) cout << "Neighbor " << max_i + 1 - i << " is in V_1" << endl;
			SG[neighbors[max_i + 1 - i].first].is_in_V_2 = false;
		}
		else {
			if (DEBUG_LBL >= 3) cout << "Neighbor " << max_i + 1 - i << " is in V_2" << endl;
			SG[neighbors[max_i + 1 - i].first].is_in_V_2 = true;
			m_j -= SG[neighbors[max_i + 1 - i].first].w;
		}
	}
	delete A;
	delete K_1;
	delete K_2;
	delete PATH;
	delete S_1;
	delete S_2;
	delete H;			

	return 1;
}

int sg_label_computer::partition_hybrid_t(edge_vector& n_e, sg_vertex_d vd, int n_n_e, int p_w, int best, sg_graph& SG) {
	if ( DEBUG_PTRAV >= 3 )
		cout << "		Partitioning for hybrid method!" << endl;
	long combinations = pow(2.0,n_n_e), i = 0, combi = 0, best_combi;
	int j = 0, best_cost = best, max_cost, max_i;
	sg_edge_d ed;
	
	for (i = 0; i < combinations; i++) {
		combi = i;
		for (j = 0; j < n_n_e; j++) {
			ed = n_e[j].first;
			SG[ed].is_in_V_2 = combi%2;
			combi = combi >> 1;
		}
		max_cost = find_max_cost_hybrid( max_i, n_e, p_w, n_n_e, SG);
		if (max_cost < best_cost) {
			best_cost = max_cost;	 
			best_combi = i;
		}
	}
	// Make sure the graph remembers the best partitioning
	combi = best_combi;
	for (j = 0; j < n_n_e; j++) {
		ed = n_e[j].first;
		set_is_in_V_2( vd, ed, combi%2, SG);
		combi = combi >> 1;
	}
	return best_cost;
}

//*** (end)

//******* Find Starting vertex ******* (fold)

sg_vertex_d sg_label_computer::get_best_start_vd( int& min_cost, int l_id, sg_graph& SG ) {
	int cost = 0, c_degree, min_degree;
	min_cost = -1;
	sg_vertex_d best_vd;
	sg_vertex_it i, i_end;
	tie( i, i_end ) = vertices(SG);
	
	if ( DEBUG_SHOW_START_VERTEX_COMP >= 1 )
		cout << " +++ Checking for best starting vertex " << endl;
	
	for ( ; i != i_end ; ++i ) {
		if ( SG[*i].mst_degree >= 1 ) {
			if ( DEBUG_SHOW_START_VERTEX_COMP >= 2 )
				cout << " Checking vertex " << *i << " as starting point" << endl;
			
			cost = start_cost( *i , l_id, SG );
			c_degree = degree( *i, SG );
			
			if ( DEBUG_SHOW_START_VERTEX_COMP >= 2 )
				cout << " - cost is " << cost << endl;
			
			if ( min_cost == -1 || min_cost > cost || ( min_cost == cost && min_degree > c_degree )) {
				min_cost = cost;
				min_degree = c_degree;
				best_vd = *i;
			}
			
		}
	}
	return best_vd;
}

int sg_label_computer::start_cost( sg_vertex_d vd, int l_id, sg_graph& SG) {
	sg_o_edge_it i, i_end;
	int e_n_n = 0;
	edge_vector e_n( SG[vd].mst_degree);
	
	// compute rho for all neighbors (e_n[].second)
	tie( i, i_end ) = out_edges( vd, SG );
	for ( ; i != i_end ; ++i ) {
		if ( SG[*i].is_in_mst == 1 ) {
			e_n[ e_n_n ].first = *i;
			e_n[ e_n_n ].second = SG.get_label( vd, *i, l_id, 1) - SG[*i].w;
			e_n_n++;
		}
	}
	
	// compute the vertex weight for clearing
	if ( l_id == 4 ) {
		int n_edges = 0, sweep_cost = 0;
		int min_sweep_cost = std::numeric_limits<int>::max();
		tie( i, i_end ) = out_edges( vd, SG );
		for ( ; i != i_end ; ++i ) {
			sweep_cost = SG[vd].ww[n_edges] + SG[*i].w;
			if ( sweep_cost < min_sweep_cost ) {
				min_sweep_cost = sweep_cost;
				SG[vd].min_sweep_dir = n_edges;
			}
			++n_edges;
		}
		SG[vd].w = min_sweep_cost;
	}
	
	edge_vector::iterator n_start = e_n.begin(), n_end = e_n.end();
	sort( n_start, n_end, edge_comparison);
	
	int max_i, max_label;
	int max_cost = find_max_cost(max_i, max_label, e_n, e_n_n, SG);
	
	//***For hybrid type strategies
	//partition if necessary and then update the maximum cost
	int max_k = max_label - e_n[max_i].second;
	int penalty_w = 0;
	if ( l_id == 1 && e_n_n > 1)
		partition_hybrid_h( e_n, e_n_n, penalty_w, max_k, max_i, max_label, SG);
	else if ( l_id == 2 && e_n_n > 1)
		partition_hybrid_t( e_n, vd, e_n_n, penalty_w, max_cost, SG );
	if ( (l_id == 1 || l_id == 2) && e_n_n > 1)
		max_cost = find_max_cost_hybrid( max_i, e_n, penalty_w, e_n_n, SG );
	// for hybrid end
	
	max_cost =  (max_cost > SG[vd].w) ? max_cost : SG[vd].w;
	return max_cost;
}

//*** (end)

//******* Helpers ******* (fold)

bool edge_comparison (const sorting_edge& a, const sorting_edge& b) {
	//WE ARE INVERTING THIS s.t. sort sorts DESC
	return (a.second > b.second);
};

int sg_label_computer::get_mst_degree( sg_vertex_d vd, sg_graph& SG ) {
	int mst_degree = 0;
	sg_o_edge_it	out_edge_it, out_edge_it_end;
	tie( out_edge_it, out_edge_it_end) = out_edges( vd, SG );
	for ( ; out_edge_it != out_edge_it_end ; ++out_edge_it ) {
		if ( SG[*out_edge_it].is_in_mst == true )
			++mst_degree;
	}
	return mst_degree;
}

/* andreas
 * 
 * Returns the vertex we consider as the first, i.e. to which the
 * label[0] belongs.
 * First is currently the smaller vertex, i.e. we consider
 * first to second <==> smaller to larger
 */

sg_vertex_d sg_label_computer::get_first( sg_edge_d ed, sg_graph& SG) {
	sg_vertex_d vd, vd2;
	vd = source( ed, SG );
	vd2 = target( ed, SG );
	if ( int(vd) < int(vd2) )
		return vd;
	else
		return vd2;
}

sg_vertex_d sg_label_computer::get_second( sg_edge_d ed, sg_graph& SG) {
	sg_vertex_d vd, vd2;
	vd = source( ed, SG );
	vd2 = target( ed, SG );
	if ( int(vd) < int(vd2) )
		return vd2;
	else
		return vd;
}

/* 
 * Collect all adjacent edges of a vertex, except ed.
 */
int 
sg_label_computer::
collect_neighbors(sg_vertex_d vd, sg_edge_d ed, edge_vector& e_n, int l_id, sg_graph& SG) {
	int i = 0;
	sg_o_edge_it oe_it, oe_it_end;
	tie(oe_it, oe_it_end) = out_edges( vd, SG );
	for (; oe_it != oe_it_end ; oe_it++ ) {
		if ( *oe_it != ed && SG[*oe_it].is_in_mst == true) {
			e_n[i].first = *oe_it;
			e_n[i].second = SG.get_label( vd, *oe_it, l_id, 1) - SG[*oe_it].w;
			i++;
		}
	}
	return i;
}

/* andreas
 * 
 * Needed (reasoning simlar than to get_label) to remember the
 * partitioning results for the various directions
 */
void sg_label_computer::set_is_in_V_2( sg_vertex_d vd, sg_edge_d ed, bool value, sg_graph& SG ) {
	if ( vd == get_first( ed, SG ) )
		SG[ed].is_in_V_2_perm[0] = value;
	else if ( vd == get_second( ed, SG ) )
		SG[ed].is_in_V_2_perm[1] = value;
	else
		DEBUG_1("ERROR: in set_is_in_V_2, invalid vd and ed combination");
	return;
}

//*** (end)

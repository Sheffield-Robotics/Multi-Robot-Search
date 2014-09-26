#include "sg_strategy.h"

/* Author: Andreas Kolling
 *
 * Constructor that builds the strategy based on hybrid labels
 */
sg_strategy::sg_strategy( sg_graph* the_graph , sg_vertex_d start_v, int label_id ) {
	private_debug = 0;
	sg_o_edge_it out_edge_it, out_edge_it_end;
	sg_adj_it    adj_it,      adj_it_end;
	sg_edge_d    dummy;
	l_id = label_id;
	visit_vertex( the_graph, start_v, dummy, 0, 1, label_id);
}

void sg_strategy::visit_vertex( sg_graph* the_graph, sg_vertex_d current_v, 
                                sg_edge_d current_e, 
                                bool V2_vertex, bool starting, int label_id) {
	sg_strategy_step new_step;
	sg_edge_d ed;
	int di, i;
	bool is_in_V_2;
	// a culprit //( (*the_graph)[current_v].mst_degree - 1);
	edge_vector e_nei = collecting_neighbors( the_graph, current_v, 
	                                          current_e, starting );
	int n_neigh = e_nei.size();
	// a culprit
	if ( private_debug >= 1 )
		cout << " Visiting " << current_v << endl;

	if ( (*the_graph)[current_v].mst_degree == 1 && starting == false ) {
		// Leaf: add it to the sequence
		if ( private_debug >= 2 )
			cout << "	 Leaf: adding it to sequence " << endl;

		new_step.new_vertex = current_v;
		if ( V2_vertex == false && !(label_id == 3 || label_id == 4) )		// vertex is in V_1 block edge
			new_step.new_blocks.push_back(current_e);
		else												// vertex is in V_2 release edge
			new_step.freed_blocks.push_back(current_e);
		check_mst_edge_blocks( the_graph, current_v , current_e, new_step);
		if ( private_debug >= 2 )
			cout << "	 Pushing new_step " << endl;
		sequence.push_back(new_step);
		if ( private_debug >= 2 )
			cout << "	 Pushing new_step done " << endl;
	}
	else {
		if ( private_debug >= 2 )
			cout << "	 Collecting neighbors " << endl;

		// NOTICE: during the cost computation it did not matter that the non-cont. and cont. labels
		// have opposite order since the cost equations are identical. For the selection of the order
		// of vertices, it does, however matter which order we're processing them.

		// 1. clear all non-cont. vertices (in V_1)
		if ( !( label_id == 3 || label_id == 4 ) ) {
			for ( i = 0 ; i < n_neigh ; i++ ) {
				ed = e_nei[i].first; // edge towards neighbor
				di = ( current_v == the_graph->get_first(ed) ) ? 1 : 0;
				is_in_V_2 = (*the_graph)[ed].is_in_V_2_perm[di];
				if ( label_id == 0 || is_in_V_2 == false ) // cleared first
					visit_vertex( the_graph, the_graph->get_neighbor(current_v, ed), ed, is_in_V_2, false, label_id);
			}
		}
		// 2. clear center vertex
		if ( private_debug >= 2 )
			cout << "	 Clearing center " << current_v << endl;

		new_step.new_vertex = current_v;
		if ( (label_id == 3 || label_id == 4) && starting == false)
			new_step.freed_blocks.push_back( current_e );
		for ( i = 0 ; i < n_neigh ; i++ ) {
			ed = e_nei[i].first;	// edge towards neighbor
			di = ( current_v == the_graph->get_first(ed) ) ? 1 : 0;
			is_in_V_2 = (*the_graph)[ed].is_in_V_2_perm[di];
			if ( is_in_V_2 == true || (label_id == 3 || label_id == 4) ) {
				// cleared later --> block edge now
				if ( private_debug >= 2 )
					cout << "	 --> add to new_blocks " << ed << endl;
				new_step.new_blocks.push_back( ed );
			}
			else {										 // cleared already --> release after clear
				new_step.freed_blocks.push_back( ed );
			}
		}

		check_mst_edge_blocks( the_graph, current_v , current_e, new_step);
		sequence.push_back(new_step);

		// 3. clear cont. part V_2 IN REVERSE ORDER!!!
		if ( !( label_id == 0 ) ) {
			if ( private_debug >= 2 )
				cout << "	 Stepping into neighbors " << n_neigh << " for " << current_v << endl;
			for ( i = n_neigh-1 ; i >= 0 ; --i ) {
				ed = e_nei[i].first; // edge towards neighbor
				di = ( current_v == the_graph->get_first(ed) ) ? 1 : 0;
				is_in_V_2 = (*the_graph)[ed].is_in_V_2_perm[di];
				if ( label_id == 3 || label_id == 4 || is_in_V_2 == true ) // cleared last
					visit_vertex( the_graph, the_graph->get_neighbor(current_v, ed), ed, is_in_V_2, false, label_id);
				if ( private_debug >= 2 )
					cout << " Back from visit	 " << current_v << endl;
			}
			if ( private_debug >= 2 )
				cout << " Out of neighbor loop " << current_v << endl;
		}
		if ( private_debug >= 2 )
			cout << " Out of if leaf else notleaf statement " << current_v << endl;
		// done
	}
	if ( private_debug >= 2 )
		cout << " EXITING	 from	 " << current_v << endl;
}


bool sg_strategy::already_in( sg_vertex_d vd ) {
	sg_strategy_seq::iterator i, end = sequence.end();
	for ( i = sequence.begin() ; i != end ; i++ ) {
		if ( i->new_vertex == vd )
			return true;
	}
	return false;
}

void sg_strategy::check_mst_edge_blocks( sg_graph* the_graph, sg_vertex_d vd, sg_edge_d current_e, sg_strategy_step& new_step ) {
	if ( private_debug >= 2 )
		cout << "	 checking mst_edge_blocks " << endl;
	if (out_degree( vd ,*the_graph ) > 1) {
		// depending on whether the other side is already clear
		// we have to add a block or remove one
		sg_vertex_d vd2;
		sg_o_edge_it oe_it, oe_it_end;
		tie(oe_it, oe_it_end) = out_edges( vd, *the_graph );
		for (; oe_it != oe_it_end ; oe_it++ ) {
			if ( *oe_it != current_e && (*the_graph)[*oe_it].is_in_mst == false ) {
				if ( private_debug >= 2 )
					cout << "	 found NON-MST Edge " << endl;
				// pick the other side of the edge
				vd2 = the_graph->get_neighbor( vd, *oe_it);
				// if vd is already in sequence it is already clear
				// otherwise add a block
				if ( already_in(vd2) )
					new_step.freed_blocks.push_back( *oe_it );
				else
					new_step.new_blocks.push_back( *oe_it );
			}
		}
	}
	if ( private_debug >= 2 )
		cout << "	 ...done " << endl;

}

edge_vector sg_strategy::collecting_neighbors( sg_graph* the_graph, sg_vertex_d current_v, sg_edge_d current_e, bool starting) {
	edge_vector e_nei;
	int n_neigh = 0;
	sg_o_edge_it oe_it, oe_it_end;
	tie(oe_it, oe_it_end) = out_edges( current_v, *the_graph );
	for ( ; oe_it != oe_it_end ; oe_it++ ) {
		if ( (starting == true || *oe_it != current_e) && (*the_graph)[*oe_it].is_in_mst == true) {
			sorting_edge sort_edge;
			sort_edge.first = *oe_it;
			sort_edge.second = the_graph->get_label( current_v, *oe_it, l_id, 1) - (*the_graph)[*oe_it].w;
			e_nei.push_back( sort_edge );
		}
	}
	// sort neighbors e_nei
	edge_vector::iterator n_start = e_nei.begin(), n_end = e_nei.end();
	sort( n_start, n_end, edge_comparison);
	return e_nei;
}

void sg_strategy::cout_strategy() {
	sg_strategy_seq::iterator i;
	sg_edge_vector::iterator j;
	cout << " ------STRATEGY--------" << endl;
	for ( i = sequence.begin() ; i != sequence.end() ; ++i ) {
		cout << " --------------------" << endl;
		j = i->new_blocks.begin();
		cout << " New Blocks: ";
		for ( ; j != i->new_blocks.end() ; ++j ) {
			cout << (*j) << " ";
		}
		cout << endl;
		cout << " V = " << i->new_vertex << endl;
		j = i->freed_blocks.begin();
		cout << " Freed Blocks: ";
		for ( ; j != i->freed_blocks.end() ; ++j ) {
			cout << (*j) << " ";
		}
		cout << endl;

	}
	cout << " --------------------" << endl;
}

sg_vertex_d sg_strategy::get_vertex( sg_graph* the_graph, int step, sg_vertex_d& from ) {
	
	sg_edge_d   e = sequence[step].freed_blocks[0];
	sg_vertex_d v = sequence[step].new_vertex;
	if ( v == source(e, *the_graph ) )
		from = target(e, *the_graph );
	else
		from = source(e, *the_graph );
	return v;
}

sg_vertex_d sg_strategy::get_vertex( int step ) {
	return sequence[step].new_vertex;
}

sg_strategy_step* sg_strategy::get_step( int step ) {
	return &(sequence[step]);
}

int sg_strategy::size() {
	return sequence.size();
}

#ifndef SG_EDGE
#define SG_EDGE

/*
	The basic edge of a surveillance graph with labels attached to it
	hybrid and noncontiguous labels are computed.
	Hybrid labels are based on the heuristic and once on the NP true value
	
	An edge has two labels, once for each direction. The first label in the array
	corresponds to the direction from 'coming_from' to the other end of the edge.
*/

#include "CGAL_types.h"

// current labels include non_contiguous, hybrid heuristic and hybrid labels
// contiguous and contiguous_mod
#define NUMBER_OF_LABELS 5
enum label_t { NONCONTIGUOUS, 
               HYBRID_HEURISTIC, 
               HYBRID_TRUE, 
               CONTIGUOUS, 
               CONTIGUOUS_MOD };

struct sg_edge
{
	VD_Halfedge e;
	int w;      // edge weight  
	int ww[2];
	SDG::Point_2 crit_p[2];
	int w_inv;  // inverted edge weight for MST computation
	bool is_in_mst;         // Is edge in minimum spanning tree?
	int x, y;               // Position in a sg_map
	// Two labels, one for each direction.
	// Directions are either from 
	// -) first to second  <=> [0]
	// -) second to first  <=> [1]
	// 
	// A label from first to second represents robots coming
	// from the first and entering the second vertex
	// These labels should be initialized to -1
	int label[ NUMBER_OF_LABELS ][2];
	bool came_from_here;
	// for hybrid methods 
	bool is_in_V_2_perm[2]; // For each direction: is edge in V_2?  
	    
	bool is_in_pipe;        // Helper for label computation
	bool is_in_V_2;         // Helper for temporary calcuations
	sg_edge() {
		for ( int i = 0 ; i < NUMBER_OF_LABELS ; ++i ) {
			label[i][0] = 0;
			label[i][1] = 0;
		}
		w = 0;
		ww[0] = 0;
		ww[1] = 0;
		is_in_mst = true;
		came_from_here = false;
	};
};

#endif


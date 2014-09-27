#ifndef SG_VERTEX
#define SG_VERTEX

#include "graphclear_legacy/CGAL_types.h"
#include "graphclear_legacy/define.h"

struct sg_vertex
{
	VD_Vertex v;
	int w;          // weight of vertex
	int ww[SG_GRAPH_MAX_VERTEX_DEGREE]; // directional weight of vertex
	SDG::Point_2 crit_p[SG_GRAPH_MAX_VERTEX_DEGREE][3];
	int min_sweep_dir;
	int mst_degree; // degree of vertex w.r.t. to MST
	int degree;     // degree of vertex
	int s;          // save sweep cost
	int x, y;       // position in a sg_map
	int robot_id;
	int robot_id2;
	bool backtracked;
	bool explored;
	bool is_left;
	double true_x, true_y;
	// Bounding box for the area corresponding to vertex
	int min_x;
	int min_y;
	int max_x;
	int max_y;
	bool alive;
	int bbox_cost;      // cost to clear the bounding box
	int max_clearance;  // ?
	sg_vertex() {
		SDG::Point_2 P1(-1,-1);
		for ( int i = 0 ; i < SG_GRAPH_MAX_VERTEX_DEGREE ; ++i ) {
			for ( int j = 0; j < 3; ++j ) {
				crit_p[i][j] = P1;
			}
			ww[i] = 0;
			w = 0;
			mst_degree = 0;
			degree = 0;
			min_sweep_dir = 0;
		}
	};
};

#endif


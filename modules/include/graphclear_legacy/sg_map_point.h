#ifndef SG_MAP_POINT
#define SG_MAP_POINT

class sg_map_point {
	public:
		int x;
		int y;
		bool occ;
		int dist_occ;
		sg_map_point(int new_x, int new_y);
		sg_map_point(int new_x, int new_y, bool new_occ, int new_dist_occ);
		bool operator< (const sg_map_point &rhs);
};

bool sg_map_point_comparison_less (const sg_map_point& a, const sg_map_point& b);

#endif

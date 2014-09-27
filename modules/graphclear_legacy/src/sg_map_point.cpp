#include "graphclear_legacy/sg_map_point.h"

sg_map_point::sg_map_point(int new_x, int new_y) {
	x = new_x;
	y = new_y;
	occ = 1;
	dist_occ = 0;
}

sg_map_point::sg_map_point(int new_x, int new_y, bool new_occ, int new_dist_occ) {
	x = new_x;
	y = new_y;
	occ = new_occ;
	dist_occ = new_dist_occ;
}

bool sg_map_point::operator<(const sg_map_point &rhs) {
	if (this->x < rhs.x)
		return true;
	else if (this->x == rhs.x && this->y < rhs.y) 
		return true;
	else
		return false;
}

bool sg_map_point_comparison_less (const sg_map_point& a, const sg_map_point& b) {
	return (a.dist_occ < b.dist_occ);
};
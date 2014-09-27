#include "graphclear_legacy/wave_point.h"

wave_point::wave_point(int new_x, int new_y) {
	x = new_x;
	y = new_y;
	dir = 0;
	cost = 0;
}

wave_point::wave_point(int new_x, int new_y, int new_dir, float new_cost) {
	x = new_x;
	y = new_y;
	dir = new_dir;
	cost = new_cost;
}

wave_point::wave_point(int new_x, int new_y, int new_dir, float new_cost, int cause_i, int cause_j) {
	x = new_x;
	y = new_y;    
	c_i = cause_i;
	c_j = cause_j;
	dir = new_dir;
	cost = new_cost;
}



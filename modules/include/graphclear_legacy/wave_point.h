#ifndef SG_WAVE_POINT
#define SG_WAVE_POINT

class wave_point {
	public:
		int x;
		int y;
		int c_i;
		int c_j;
		int dir; //direction of wave propagation
		float cost;
		wave_point(int new_x, int new_y);
		wave_point(int new_x, int new_y, int new_dir, float new_cost);
		wave_point(int new_x, int new_y, int new_dir, float new_cost, int cause_i, int cause_j);
		bool comparison(const wave_point s1, const wave_point s2);
};

struct ltstr
{
  bool operator()(const wave_point& s1, const wave_point& s2) const
  {
    if (s1.cost > s2.cost)
		return true;
	else
		return false;
  }
};


#endif

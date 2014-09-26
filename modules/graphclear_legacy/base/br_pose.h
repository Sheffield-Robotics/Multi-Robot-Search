/*
* A simple pose class for the base robot
*/

#ifndef BR_POSE_H
#define BR_POSE_H

#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

class br_pose {
private:
  
public:
	int pose_dim;
	float pose[4];
	
	br_pose();
	
	br_pose(int dim);
	
	br_pose(const br_pose &y);
	
	br_pose( br_pose &robot_pose, double rel_bearing , double dist );
	
	~br_pose();
	
	br_pose& 
		operator= (const br_pose& y);
	bool 
		operator== (const br_pose& y);
	br_pose 
		operator+ (const br_pose& y);
	br_pose
		operator- (const br_pose& y);
	
	br_pose 
		operator* (const float& y);
	br_pose 
		operator* (const double& y);
	float& 
		operator[] (int n) { return pose[n];};
	
	friend std::ostream &operator<< ( std::ostream &stream, br_pose a_pose ) {
		stream << a_pose.pose[0] << ":" << a_pose.pose[1] << " a=" << a_pose.pose[2];
		return stream;
	};

	void
		rotate_as_vector( double angle );
	void 
		rotate_yaw( double angle );
	void 
		normalize_as_vector( double distance );
	void 
		put_vector_orientation();
	float 
		get_angle( br_pose& y );
	
	float 
		distance( br_pose& y );
	float 
		distance( );
	

	bool 
		equal (const br_pose& y, float threshold, int dimensions);
	br_pose 
		real_to_pixel( float resolution, int max_pix[] );
	br_pose
		pixel_to_real( float resolution, int max_pix[] );
	br_pose 
		real_to_pixel_inv( float resolution, int max_pix[] );
	br_pose 
		pixel_to_real_inv( float resolution, int max_pix[] );
	br_pose 
    displace( int sub_id );
};

typedef std::vector< br_pose > 
  br_pose_vector;
typedef std::vector< br_pose_vector* > 
  br_pose_matrix;


#endif

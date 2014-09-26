/* standard pose */

#include "br_pose.h"

br_pose::br_pose(int dim) {
	if ( dim < 5) {
		pose_dim = dim;
		for( int i = 0; i < pose_dim ; ++i )
			pose[i] = 0;
	}
}

br_pose::br_pose() {
	pose_dim = 4;
	pose[0] = 0;pose[1] = 0;pose[2] = 0;pose[3] = 0;
}

br_pose::~br_pose() {
}

br_pose::br_pose(const br_pose &y) {
	pose_dim = y.pose_dim;
	for ( int i = 0; i < pose_dim; ++i )
		pose[i] = y.pose[i];
}

br_pose::br_pose( br_pose &robot_pose, double rel_bearing , double dist )
{
	// create a new pose from an old pose and a relative bearing and distance
	double global_bearing = robot_pose[2] + rel_bearing;
	if ( global_bearing > M_PI )
		global_bearing -= 2*M_PI;
	else if ( global_bearing < -M_PI )
		global_bearing += 2*M_PI;
	pose_dim = 4;//no point is given max dimension
	if ( robot_pose.pose_dim == 3 || robot_pose.pose_dim == 4 ) {
		pose[0] = robot_pose.pose[0] + cos( global_bearing ) * dist;
		pose[1] = robot_pose.pose[1] + sin( global_bearing ) * dist;
		pose[2] = 0;
		pose[3] = 0;
	}
}

void br_pose::rotate_as_vector( double angle ) {
	// interpret the pose as a 2d vector and rotate it by 'angle'
	// 
	float old_p_x = pose[0];
	pose[0] = cos( angle ) * pose[0] - sin( angle ) * pose[1];
	pose[1] = sin( angle ) * old_p_x + cos( angle ) * pose[1];	
}

void br_pose::rotate_yaw( double angle ) 
{
	pose[2] += angle;
	if ( pose[2] > M_PI )
		pose[2] -= 2*M_PI;
	else if ( pose[2] < -M_PI )
		pose[2] += 2*M_PI;
}

void br_pose::normalize_as_vector( double distance ) {
	double normalizer =	 distance / sqrt( (pow(pose[0],2) + pow(pose[1],2)) );
	pose[0] *= normalizer;
	pose[1] *= normalizer;
}

float br_pose::get_angle( br_pose& y ) 
{
	this->normalize_as_vector( 1 );
	y.normalize_as_vector( 1 );
	return acos( pose[0]*y[0] + pose[1]*y[1] );
}

void br_pose::put_vector_orientation() 
{
	// compute the bearing/angle of the vector and put it into pose[2] 
	// A HACK!!!
	double dist = sqrt( (pow(pose[0],2) + pow(pose[1],2)) );
	if ( dist == 0 )
		pose[2] = 0;
	pose[2] = acos( pose[0] / dist );
	if ( pose[1] < 0 )
		pose[2] = - pose[2];
}


br_pose& br_pose::operator= (const br_pose& y) {
	if ( &y != this ) {
		this->pose_dim = y.pose_dim;
		for ( int i = 0; i < pose_dim; ++i )
			this->pose[i] = y.pose[i];
	}
	return *this;
}

bool br_pose::operator== (const br_pose& y) {
	if (y.pose_dim != this->pose_dim)
		return false;
	for ( int i = 0; i < pose_dim; ++i ) {
			if ( this->pose[i] != y.pose[i] )
				return false;
	}
	return true;
}

br_pose br_pose::operator+ (const br_pose& y) {
	br_pose pose_p( this->pose_dim );	 
	if (y.pose_dim != this->pose_dim)
		return pose_p;
	for ( int i = 0; i < pose_dim; ++i ) {
		pose_p.pose[i] = this->pose[i] + y.pose[i]; 
	}
	return pose_p;
}

br_pose br_pose::operator- (const br_pose& y) {
	int new_dim = 0;
	if ( this->pose_dim > y.pose_dim ) 
		new_dim = y.pose_dim;
	else
		new_dim = this->pose_dim;
	br_pose pose_p( new_dim );
	//cout << " POSE DIM " << pose_p.pose_dim << endl;
	for ( int i = 0; i < pose_p.pose_dim ; ++i ) {
		pose_p.pose[i] = this->pose[i] - y.pose[i]; 
	}
	return pose_p;
}

br_pose br_pose::operator* (const float& y) {
	br_pose pose_p( this->pose_dim );	 
	for ( int i = 0; i < pose_dim; ++i ) {
		pose_p.pose[i] = this->pose[i] * y;
	}
	return pose_p;
}

br_pose br_pose::operator* (const double& y) {
	br_pose pose_p( this->pose_dim );	 
	for ( int i = 0; i < pose_dim; ++i ) {
		pose_p.pose[i] = this->pose[i] * y;
	}
	return pose_p;
}

float br_pose::distance( br_pose& y ) {
	return sqrt(pow(pose[0] - y[0],2) + pow(pose[1] - y[1],2));
}

float br_pose::distance() {
	return sqrt(pow(pose[0],2) + pow(pose[1],2));
}


bool br_pose::equal (const br_pose& y, float threshold, int dimensions) {
	float prod = 0, sum = 0;
	if (y.pose_dim < dimensions || this->pose_dim < dimensions)
		return false;
	for ( int i = 0; i < dimensions; ++i ) {
		prod = this->pose[i] - y.pose[i];
		if ( i == 2 )
			sum += prod * prod / 3;
		else
			sum += prod * prod;
	}
	if ( sqrt(sum) > threshold ) {
		return false;
	}
	return true;
}

/* Author: Andreas Kolling
 * 
 * Makes the assumption that the real map has its origin in the middle
 */

br_pose br_pose::real_to_pixel( float resolution, int max_pix[] ) {
	br_pose pix_pos( this->pose_dim );
	pix_pos.pose[0] = this->pose[0] / resolution + 0.5 * max_pix[0];
	pix_pos.pose[1] = -this->pose[1] / resolution + 0.5 * max_pix[1];
	if ( this->pose_dim > 2 ) {
		pix_pos.pose[2] = this->pose[2];
	}
	if ( this->pose_dim > 3 ) {
		pix_pos.pose[3] = this->pose[3];
	}
	
	return pix_pos;
}


br_pose br_pose::pixel_to_real( float resolution, int max_pix[] ) {
	br_pose real_pos( this->pose_dim );
	real_pos.pose[0] =	 ( this->pose[0] - 0.5 * max_pix[0]) * resolution;
	real_pos.pose[1] = - ( this->pose[1] - 0.5 * max_pix[1]) * resolution;
	if ( this->pose_dim > 2 ) {
		real_pos.pose[2] = this->pose[2];
	}
	if ( this->pose_dim > 3 ) {
		real_pos.pose[3] = this->pose[3];
	}
	return real_pos;
}

br_pose br_pose::real_to_pixel_inv( float resolution, int max_pix[] ) {
	br_pose pix_pos( this->pose_dim );
	pix_pos.pose[1] = this->pose[0] / resolution + 0.5 * max_pix[0];
	pix_pos.pose[0] = -this->pose[1] / resolution + 0.5 * max_pix[1];
	if ( this->pose_dim > 2 ) {
		pix_pos.pose[2] = this->pose[2];
	}
	if ( this->pose_dim > 3 ) {
		pix_pos.pose[3] = this->pose[3];
	}
	return pix_pos;
}


br_pose br_pose::pixel_to_real_inv( float resolution, int max_pix[] ) {
	br_pose real_pos( this->pose_dim );
	real_pos.pose[0] =	 ( this->pose[1] - 0.5 * max_pix[1]) * resolution;
	real_pos.pose[1] = - ( this->pose[0] - 0.5 * max_pix[0]) * resolution;
	return real_pos;
}

br_pose br_pose::displace( int sub_id ) {
	float displ_fact = 0.4; // Displacement by 0.25 meter
	float mod_angle;
	if ( sub_id%2 == 0 ) {
		mod_angle = 3.14159265 / 2;
	}
	else {
		mod_angle = - 3.14159265 / 2;
	}
	mod_angle = this->pose[2] + mod_angle;
	if ( mod_angle > 3.14159265 )
		mod_angle = mod_angle - 2*3.14159265;
	else if ( mod_angle < -3.14159265 )
		mod_angle = mod_angle + 2*3.14159265;
	
	br_pose returnpose;
	returnpose.pose[0] = this->pose[0] + displ_fact * cos( mod_angle );
	returnpose.pose[1] = this->pose[1] + displ_fact * sin( mod_angle );
	returnpose.pose[2] = this->pose[2];
	return returnpose;
}

#include "utilities/math/vector3d.h"

double& Vector3d::operator[] (int index){
	if(index == 0)
		return x;
	else if(index == 1)
		return y;
	else 
		return z;
		
}

void Vector3d::print() const {
	std::cout << *this << std::endl;
}

std::ostream& operator<<(std::ostream &os, const Vector3d &v){
    return os << "(" << v.x << ", " << v.y << ", " << v.z <<")";
}

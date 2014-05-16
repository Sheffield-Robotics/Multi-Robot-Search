#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <iostream>


class Vector3d {
	public:
		Vector3d() : x(0), y(0), z(0) {}
		Vector3d(double ix, double iy, double iz) : x(ix), y(iy), z(iz) {}

		void print() const;

		double x,y,z;             /* the usual 3-space position of a vertex */
		Vector3d operator + (const Vector3d &arg) const {
			return Vector3d(x + arg.x, y + arg.y, z + arg.z);
		}

		double& operator [] (int);
};

std::ostream& operator<<(std::ostream &os, const Vector3d &v);


#endif

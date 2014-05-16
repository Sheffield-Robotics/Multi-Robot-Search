#ifndef VECTOR4_H
#define VECTOR4_H

#include "utilities/math/vector.h"
#include "utilities/math/vector3.h"

#include <iostream>
#include <cmath>

/**
 * \brief a 4d vector, e.g. used as homogeneous coordinates
 */
template <typename X>
class DVector4 : public DVector<X>
{
  public:
    DVector4() : DVector<X>(4) {}

    //! constructor taking 4 coordinates, usually the fourth is the scale
    DVector4(const X& x, const X& y, const X& z, const X& w = 1);
    DVector4(const DVector4<X>&);
    DVector4(const DVector<X>&);

    void get3DCoords(X& x, X& y, X& z) const;
    DVector3<X> get3DPoint() const;

    DVector4& operator=(const DVector<X>& );

};

typedef DVector4<double> Vector4;
typedef DVector4<float>  Vector4f;

template <class X>
DVector4<X>& DVector4<X>::operator=(const DVector<X>& m) {
  if (this == &m)
    return *this;
  if (m.size() != 4)
    throw std::runtime_error("vector size != 4");
  if (!--(*this->shares)) {
    delete [] this->elems;
    delete this->shares;
  }
  this->shares = const_cast<int*>(m.getShares());
  this->elems  = const_cast<X*>(m.getData());
  this->_size  = m.size();
  (*this->shares)++;
  return *this;
}

template <class X>
DVector4<X>::DVector4(const DVector<X>& m) : DVector<X>(m)
{
  if (m.size() != 4)
    throw std::runtime_error("vector size != 4");
}

template <class X>
DVector4<X>::DVector4(const DVector4<X>& m) : DVector<X>(m)
{
}

template <class X>
DVector4<X>::DVector4(const X& x, const X& y, const X& z, const X& w) : DVector<X>(4)
{
  this->elems[0] = x;
  this->elems[1] = y;
  this->elems[2] = z;
  this->elems[3] = w;
}

template <class X>
void DVector4<X>::get3DCoords(X& x, X& y, X& z) const
{
  x = this->elems[0] / this->elems[3];
  y = this->elems[1] / this->elems[3];
  z = this->elems[2] / this->elems[3];
}

template <class X>
DVector3<X> DVector4<X>::get3DPoint() const
{
  DVector3<X> out;
  get3DCoords(out[1], out[2], out[3]);
  return out;
}

#endif

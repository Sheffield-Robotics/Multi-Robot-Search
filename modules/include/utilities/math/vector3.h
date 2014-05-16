#ifndef VECTOR3_H
#define VECTOR3_H

#include "vector.h"

#include <iostream>
#include <cmath>

/**
 * \brief a 3d vector
 */
template <typename X>
class DVector3 : public DVector<X>
{
  public:
    DVector3() : DVector<X>(3) {}

    //! constructor taking 3 coordinates
    DVector3(const X& x, const X& y, const X& z);
    DVector3(const DVector3<X>&);
    DVector3(const DVector<X>&);

    DVector3& operator=(const DVector<X>& );

    //! return the x coordinate
    const X& x() const { return (*this)[0];}
    //! return the y coordinate
    const X& y() const { return (*this)[1];}
    //! return the z coordinate
    const X& z() const { return (*this)[2];}
    X& x() { return (*this)[0];}
    X& y() { return (*this)[1];}
    X& z() { return (*this)[2];}

    // TODO overwrite operators to save range checks

    DVector3<X> cross(const DVector3<X>& other) const;
    void cross(const DVector3<X>& other, DVector3<X>& result) const;
    static void cross(X x1, X y1, X z1, X x2, X y2, X z2, DVector3<X>& result);
    
    inline X getSquaredEuclidianDistance(const DVector3<X>& other) const;
    inline X getEuclidianDistance(const DVector3<X>& other) const { return sqrt(getSquaredEuclidianDistance(other));}

    inline void setValues(X x, X y, X z);
    inline void setValues(const DVector3<X>& other);
};

typedef DVector3<double> Vector3;
typedef DVector3<float>  Vector3f;
typedef DVector3<int>    IVector3;

template <class X>
DVector3<X>& DVector3<X>::operator=(const DVector<X>& m) {
  if (this == &m)
    return *this;
  if (m.size() != 3)
    throw std::runtime_error("vector size != 3");
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
DVector3<X>::DVector3(const DVector<X>& m) : DVector<X>(m)
{
  if (m.size() != 3)
    throw std::runtime_error("vector size != 3");
}

template <class X>
DVector3<X>::DVector3(const DVector3<X>& m) : DVector<X>(m)
{
}

template <class X>
DVector3<X>::DVector3(const X& x, const X& y, const X& z) : DVector<X>(3)
{
  this->elems[0] = x;
  this->elems[1] = y;
  this->elems[2] = z;
}

template <class X>
void DVector3<X>::cross(const DVector3<X>& other, DVector3<X>& result) const
{
  result[0] = this->elems[1] * other.elems[2] - this->elems[2] * other.elems[1];
  result[1] = this->elems[2] * other.elems[0] - this->elems[0] * other.elems[2];
  result[2] = this->elems[0] * other.elems[1] - this->elems[1] * other.elems[0];
}

template <class X>
void DVector3<X>::cross(X x1, X y1, X z1, X x2, X y2, X z2, DVector3<X>& result) {
  result[0] = y1 * z2 - z1 * y2;
  result[1] = z1 * x2 - x1 * z2;
  result[2] = x1 * y2 - y1 * x2;
}

template <class X>
DVector3<X> DVector3<X>::cross(const DVector3<X>& other) const
{
  return DVector3<X>(this->elems[1] * other.elems[2] - this->elems[2] * other.elems[1],
		     this->elems[2] * other.elems[0] - this->elems[0] * other.elems[2],
		     this->elems[0] * other.elems[1] - this->elems[1] * other.elems[0] );
}

template <class X>
X DVector3<X>::getSquaredEuclidianDistance(const DVector3<X>& other) const {
  X* tmpElems1 = this->elems;
  X* tmpElems2 = other.elems;

  X diff = *(tmpElems2++) - *(tmpElems1++);
  X ret = diff*diff;
  diff = *(tmpElems2++) - *(tmpElems1++);
  ret += diff*diff;
  diff = *tmpElems2 - *tmpElems1;
  ret += diff*diff;

  return ret;
}

template <class X>
inline void DVector3<X>::setValues(X x, X y, X z) {
  if ((*this->shares)>1) this->detach();
  X* tmpElems1 = this->elems;
  *(tmpElems1++) = x; *(tmpElems1++) = y; *tmpElems1     = z;
}

template <class X>
inline void DVector3<X>::setValues(const DVector3<X>& other) {
  if ((*this->shares)>1) this->detach();
  X* tmpElems1 = this->elems, * tmpElems2 = other.elems;
  *(tmpElems1++) = *(tmpElems2++);
  *(tmpElems1++) = *(tmpElems2++);
  *tmpElems1     = *tmpElems2;
}

#endif

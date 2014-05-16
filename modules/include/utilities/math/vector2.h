#ifndef VECTOR2_H
#define VECTOR2_H

#include "utilities/math/vector.h"

#include <iostream>
#include <cmath>

/**
 * \brief a 2d vector
 */
template <typename X>
class DVector2 : public DVector<X>
{
  public:
    DVector2() : DVector<X>(2) {}

    //! constructor taking two coordinates
    DVector2(const X& x, const X& y);
    DVector2(const DVector2<X>&);
    DVector2(const DVector<X>&);

    DVector2& operator=(const DVector<X>& );

    //! return x coordinate
    const X& x() const {return (*this)[0];}
    //! return y coordinate
    const X& y() const {return (*this)[1];}
    X& x() {return (*this)[0];}
    X& y() {return (*this)[1];}

    inline X getSquaredEuclidianDistance(const DVector2<X>& other) const;
    inline X getEuclidianDistance(const DVector2<X>& other) const { return sqrt(getSquaredEuclidianDistance(other));}

    // TODO overwrite operators to save range checks

};

typedef DVector2<double> Vector2;
typedef DVector2<float>  Vector2f;
typedef DVector2<int>    IVector2;

template <class X>
DVector2<X>& DVector2<X>::operator=(const DVector<X>& m) {
  if (this == &m)
    return *this;
  if (m.size() != 2)
    throw std::runtime_error("vector size != 2");
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
DVector2<X>::DVector2(const DVector<X>& m) : DVector<X>(m)
{
  if (m.size() != 2)
    throw std::runtime_error("vector size != 2");
}

template <class X>
DVector2<X>::DVector2(const DVector2<X>& m) : DVector<X>(m)
{
}

template <class X>
DVector2<X>::DVector2(const X& x, const X& y) : DVector<X>(2)
{
  this->elems[0] = x;
  this->elems[1] = y;
}

template <class X>
X DVector2<X>::getSquaredEuclidianDistance(const DVector2<X>& other) const {
  X diff = *(this->elems) - *(other.elems);
  X ret = diff*diff;
  diff = *(this->elems+1) - *(other.elems+1);
  ret += diff*diff;
  return ret;
}

#endif

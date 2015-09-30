#ifndef OCTREE_VECTOR3D_HPP
#define OCTREE_VECTOR3D_HPP

#include <iostream>
#include <iomanip>

namespace octree {

/*!
 * A three-dimensional vector.
 *
 * \param T  the type of the elements in the vector.
 */
template <class T>
class vector3D {
public:
  typedef T value_type; /*!< The first template parameter (`T`) */

  /*!
   * Default constructor.
   *
   * Creates a null vector with `x`, `y` and `z` set to zero.
   */
  vector3D() : vector3D(0.0, 0.0, 0.0) {}

  /*!
   * Creates a vector.
   *
   * \param x  the value of the `x` component
   * \param y  the value of the `y` component
   * \param z  the value of the `z` component
   */
  vector3D(const T & x, const T & y, const T & z) {
    _data[0] = x;
    _data[1] = y;
    _data[2] = z;
  }

  const T & x() const { return _data[0]; }
  const T & y() const { return _data[1]; }
  const T & z() const { return _data[2]; }

  void x(const T & x) { _data[0] = x; }
  void y(const T & y) { _data[1] = y; }
  void z(const T & z) { _data[2] = z; }

  /*!
   * Gets a pointer to the elements as a coalesced array for interoperability
   * with other libraries.  The `x` component of the vector is stored in offset
   * `0`, the `y` component in offset `1` and the `z` component in offset `2`.
   * The array has length `3`.
   */
  const T * data() const { return _data; }
  T * data() { return _data; }
private:
  T _data[3];
};

template <class T>
std::ostream & operator<<(std::ostream & os, const vector3D<T> & p) {
  std::streamsize w = os.width();
  return os << "(" << std::setw(w) << p.x() << ", " <<
                      std::setw(w) << p.y() << ", " <<
                      std::setw(w) << p.z() << ")";
}

}       // namespace octree

#endif  // OCTREE_VECTOR3D_HPP

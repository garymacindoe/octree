#ifndef OCTREE_BOUNDS3D_HPP
#define OCTREE_BOUNDS3D_HPP

#include <iostream>
#include <iomanip>
#include <cassert>

#include "vector3D.hpp"

namespace octree {

/*!
 * A three-dimensional axis-aligned bounding box.
 *
 * \param T  the data type used to store the minimum and maximum x, y and z
 *             values
 */
template <class T>
class bounds3D {
public:
  typedef T value_type; /*!<  The first template parameter (`T`) */

  /*!
   * Default constructor.
   *
   * Creates a zero-width bounding box.
   */
  bounds3D() : bounds3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) {}

  /*!
   * Creates a bounding box.  The lower bound for each dimension must be less
   * than or equal to the upper bound.
   *
   * \param xmin  the lower bound of the x dimension
   * \param xmax  the upper bound of the x dimension
   * \param ymin  the lower bound of the y dimension
   * \param ymax  the upper bound of the y dimension
   * \param zmin  the lower bound of the z dimension
   * \param zmax  the upper bound of the z dimension
   */
  bounds3D(const T & xmin, const T & xmax,
           const T & ymin, const T & ymax,
           const T & zmin, const T & zmax) {
    assert(xmin <= xmax);
    assert(ymin <= ymax);
    assert(zmin <= zmax);
    _data[0] = xmin;
    _data[1] = xmax;
    _data[2] = ymin;
    _data[3] = ymax;
    _data[4] = zmin;
    _data[5] = zmax;
  }

  const T & xmin() const { return _data[0]; }
  const T & xmax() const { return _data[1]; }
  const T & ymin() const { return _data[2]; }
  const T & ymax() const { return _data[3]; }
  const T & zmin() const { return _data[4]; }
  const T & zmax() const { return _data[5]; }

  void xmin(const T & xmin) { _data[0] = xmin; }
  void xmax(const T & xmax) { _data[1] = xmax; }
  void ymin(const T & ymin) { _data[2] = ymin; }
  void ymax(const T & ymax) { _data[3] = ymax; }
  void zmin(const T & zmin) { _data[4] = zmin; }
  void zmax(const T & zmax) { _data[5] = zmax; }

  /*!
   * Gets a pointer to the bounds as a coalesced array for interoperability
   * with other libraries.  The bounds are stored in the array in the following
   * order: `xmin`, `xmax`, `ymin`, `ymax`, `zmin`, `zmax`.  The array has
   * length `6`.
   */
  const T * data() const { return _data; }
  T * data() { return _data; }
private:
  T _data[6];
};

template <class T>
std::ostream & operator<<(std::ostream & os, const bounds3D<T> & bounds) {
  std::streamsize w = os.width();
  return os << "{ " << std::setw(w) << bounds.xmin() << ", " <<
                       std::setw(w) << bounds.xmax() << ", " <<
                       std::setw(w) << bounds.ymin() << ", " <<
                       std::setw(w) << bounds.ymax() << ", " <<
                       std::setw(w) << bounds.zmin() << ", " <<
                       std::setw(w) << bounds.zmax() << " }";
}

/*!
 * Returns `true` if the point is contained within the bounds.  This function
 * treats the bounds as a closed-open interval along each dimension (i.e. the
 * point is considered to be within the bounds if for each dimension x, y and z
 * the expression `xmin <= x < xmax` evaluates as `true`).
 *
 * \param bounds  the bounds
 * \param point   the point
 *
 * \returns `true` if the point is within the bounds, `false` otherwise.
 */
template <class T>
inline bool contains(const bounds3D<T> & bounds, const vector3D<T> & point) {
  return point.x() >= bounds.xmin() && point.x() < bounds.xmax() &&
         point.y() >= bounds.ymin() && point.y() < bounds.ymax() &&
         point.z() >= bounds.zmin() && point.z() < bounds.zmax();
}

/*!
 * Calculates the centre of the bounding box.
 *
 * \param bounds  the bounding box
 *
 * \return a `vector3D` containing the coordinates of the centre.
 */
template <class T>
inline vector3D<T> centre(const bounds3D<T> & bounds) {
  return vector3D<T>((bounds.xmin() + bounds.xmax()) / 2.0,
                     (bounds.ymin() + bounds.ymax()) / 2.0,
                     (bounds.zmin() + bounds.zmax()) / 2.0);
}

}       // namespace octree

#endif  // OCTREE_BOUNDS3D_HPP

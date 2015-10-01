#ifndef OCTREE_MAP_H
#define OCTREE_MAP_H

#include <memory>
#include <utility>
#include <iterator>
#include <type_traits>

namespace octree {

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
template <class Bounds3D, class Vector3D>
struct contains {
  typedef Bounds3D volume_type;
  typedef Vector3D point_type;
  bool operator()(const Bounds3D & bounds, const Vector3D & point) {
    return point.x() >= bounds.xmin() && point.x() < bounds.xmax() &&
           point.y() >= bounds.ymin() && point.y() < bounds.ymax() &&
           point.z() >= bounds.zmin() && point.z() < bounds.zmax();
  }
};

/*!
 * Calculates the centre of the bounding box.
 *
 * \param bounds  the bounding box
 *
 * \return a `vector3D` containing the coordinates of the centre.
 */
template <class Bounds3D, class Vector3D>
struct centre {
  typedef Bounds3D volume_type;
  typedef Vector3D point_type;
  Vector3D operator()(const Bounds3D & bounds) {
    return Vector3D((bounds.xmin() + bounds.xmax()) / 2,
                    (bounds.ymin() + bounds.ymax()) / 2,
                    (bounds.zmin() + bounds.zmax()) / 2);
  }
};

/*!
 * Simple octree-backed map class.  Mostly follows the interface of `std::map`,
 * with the volumes being mapped to values and keys being points.  Rather than
 * inserting arbitrary key/value pairs into the map the map starts with an
 * initial volume which is then subdivided.  Values are stored in leaf nodes.
 *
 * \param Vector3D  the type of the points.  Aliased as
 *                    `octree_map::point_type`.
 * \param Bounds3D  the type of the volume.  Aliased as
 *                    `octree_map::volume_type`.
 * \param T         the type of object to store in the map.  Aliased as
 *                    `octree_map::mapped_type`.
 * \param Contains  a binary predicate that takes a `Bounds3D` and a `Vector3D`
 *                    as arguments and returns a `bool`.  The expression
 *                    `contains(b, p)`, where `contains` is an object of this
 *                    type, `b` is a `Bounds3D` and `p` is a `Vector3D`, shall
 *                    return `true` if `p` is considered to be within `b`.
 *                    This allows volumes to be implemented as open, closed,
 *                    half-open or half-closed intervals in each dimension.  A
 *                    point maps to a value if the volume mapped to the value
 *                    contains the point.  This defaults to
 *                    `contains<Bounds3D, Vector3D>`.
 *                    Aliased as `octree_map::volume_contains`.
 * \param Centre    a unary predicate that returns a `Vector3D` representing
 *                    the centre of a `Bounds3D` instance.  When subdividing
 *                    the tree the volumes are divided around this point.  This
 *                    defaults to `centre<Bounds3D, Vector3D>`.
 *                    Aliased as `octree_map::volume_centre`.
 * \param Alloc     Type of the allocator object used to define the storage
 *                  allocation model.  This defaults to
 *                  `std::allocator<std::pair<const Bounds3D, T>>`.
 *                  Aliased as `octree_map::allocator_type`.
 */
template <class Vector3D, class Bounds3D, class T,
          class Contains = contains<Bounds3D, Vector3D>,
          class Centre = centre<Bounds3D, Vector3D>,
          class Alloc = std::allocator<std::pair<const Bounds3D, T>>>
class map {
public:

  /*! The first template parameter (`Vector3D`) */
  typedef Vector3D point_type;

  /*! The second template parameter (`Bounds3D`) */
  typedef Bounds3D volume_type;

  /*! The third template parameter (`T`) */
  typedef T mapped_type;

  /*! `std::pair<const volume_type, mapped_type>` */
  typedef std::pair<const volume_type, mapped_type> value_type;

  /*! The fourth template parameter (`Contains`) */
  typedef Contains volume_contains;

  /*! The fifth template parameter (`Centre`) */
  typedef Centre volume_centre;

  /*! Nested function to compare elements */
  typedef void value_contains;

  /*! The sixth template parameter (`Alloc`) */
  typedef Alloc allocator_type;

  /*! `value_type &` */
  typedef value_type & reference;

  /*! `const value_type &` */
  typedef const value_type & const_reference;

  /*! `std::allocator_traits<allocator_type>::pointer` */
  typedef typename std::allocator_traits<allocator_type>::pointer pointer;

  /*! `std::allocator_traits<allocator_type>::const_pointer` */
  typedef typename std::allocator_traits<allocator_type>::const_pointer
      const_pointer;

  /*! A bidirectional iterator to `value_type` */
  typedef T * iterator;

  /*! A bidirectional iterator to `const value_type` */
  typedef T * const_iterator;

  /*! `std::reverse_iterator<iterator>` */
  typedef std::reverse_iterator<iterator> reverse_iterator;

  /*! `std::reverse_iterator<const_iterator>` */
  typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

  /*! A signed integral type, identical to
   * `std::iterator_traits<iterator>::difference_type` */
  typedef typename std::iterator_traits<iterator>::difference_type
      difference_type;

  /*! An unsigned integral type that can represent any non-negative value of
   * `difference_type` */
  typedef std::make_unsigned<difference_type> size_type;
};

}       // namespace octree

#endif

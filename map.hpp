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
  bool operator()(const Bounds3D & bounds, const Vector3D & point) const {
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
  Vector3D operator()(const Bounds3D & bounds) const {
    return Vector3D((bounds.xmin() + bounds.xmax()) / 2,
                    (bounds.ymin() + bounds.ymax()) / 2,
                    (bounds.zmin() + bounds.zmax()) / 2);
  }
};

/*!
 * Simple octree-backed map class.  Mostly follows the interface of `std::map`,
 * with the volumes being mapped to values and keys being points.  Rather than
 * inserting arbitrary key/value pairs into the map the map starts with an
 * initial volume which is then subdivided.
 *
 * \param Vector3D  the type of the points.  Aliased as
 *                    `map::point_type`.
 * \param Bounds3D  the type of the volume.  Aliased as
 *                    `map::volume_type`.
 * \param T         the type of object to store in the map.  Aliased as
 *                    `map::mapped_type`.
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
 *                    Aliased as `map::volume_contains`.
 * \param Centre    a unary predicate that returns a `Vector3D` representing
 *                    the centre of a `Bounds3D` instance.  When subdividing
 *                    the tree the volumes are divided around this point.  This
 *                    defaults to `centre<Bounds3D, Vector3D>`.
 *                    Aliased as `map::volume_centre`.
 * \param Alloc     Type of the allocator object used to define the storage
 *                  allocation model.  This defaults to
 *                  `std::allocator<std::pair<const Bounds3D, T>>`.
 *                  Aliased as `map::allocator_type`.
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
  typedef const T * const_iterator;

  /*! A tree cursor to `value_type` */
  typedef T * cursor;

  /*! A tree cursor to `const value_type` */
  typedef const T * const_cursor;

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

  /*!
   * Constructs an octree map with a single node.
   *
   * \param bounds         the bounds of the map
   * \param value          the value points within the bounds initially map to
   * \param contains       a binary predicate that takes `Bounds3D` and
   *                         `Vector3D` instances and returns `true` if the
   *                         point represented by the vector is contained within
   *                         the bounds.  May be a function pointer or a
   *                         function object.
   * \param centre         a unary predicate that takes a `Bounds3D` instance
   *                         and returns a `Vector3D` representing the centre of
   *                         the bounds.  May be a function pointer or a
   *                         function object.
   * \param alloc          Allocator object.  The container keeps and uses an
   *                         internal copy of this object.
   */
  explicit map(const volume_type & = volume_type(),
               const mapped_type & = mapped_type(),
               const volume_contains & = volume_contains(),
               const volume_centre & = volume_centre(),
               const allocator_type & = allocator_type());

  /*!
   * Constructs a map with a copy of each of the elements in `x`.
   *
   * \param x      another map object of the same type, whose contents are
   *                 copied
   * \param alloc  Allocator object.  The container keeps and uses an internal
   *                 copy of this object.
   */
  map(const map &);
  map(const map &, const allocator_type &);

  /*!
   * Constructs a container that acquires the elements of x.  If `alloc` is
   * specified and is different from `x`'s allocator, the elements are moved.
   * Otherwise, no elements are constructed (their ownership is directly
   * transferred).  `x` is left in an unspecified but valid state.
   *
   * \param x      another map object of the same type, whose contents are
   *                 moved into this object
   * \param alloc  Allocator object.  The container keeps and uses an internal
   *                 copy of this object.
   */
  map(map &&);
  map(map &&, const allocator_type &);

  /*!
   * Copies all the elements of `x` into this map.
   *
   * \param x  another map object of the same type, whose contents are copied
   */
  map & operator=(const map &);

  /*!
   * Moves the elements of `x` into this map.  `x` is left in an unspecified but
   * valid state.
   *
   * \param x  another map object of the same type, whose contents are moved
   *             into this map
   */
  map & operator=(const map &&);

  /*!
   * Destroys the octree map.
   */
  ~map();


  /*!
   * Returns an iterator pointing to the first element in the map.
   */
  iterator begin() noexcept;
  const_iterator begin() const noexcept;

  /*!
   * Returns an iterator pointing to one-past-the-end element in the map.
   *
   * The value returned shall not be dereferenced.
   */
  iterator end() noexcept;
  const_iterator end() const noexcept;

  /*!
   * Returns a reverse iterator pointing to the last element in the map.
   */
  reverse_iterator rbegin() noexcept;
  const_reverse_iterator rbegin() const noexcept;

  /*!
   * Returns a reverse iterator pointing to the theoretical element right before
   * the first element in the map.
   */
  reverse_iterator rend() noexcept;
  const_reverse_iterator rend() const noexcept;

  /*!
   * Returns a `const_iterator` pointing to the first element in the map.
   */
  const_iterator cbegin() const noexcept;

  /*!
   * Returns a `const_iterator` pointing to one-past-the-end element in the map.
   *
   * The value returned shall not be dereferenced.
   */
  const_iterator cend() const noexcept;

  /*!
   * Returns a `const_reverse_iterator` pointing to the last element in the map.
   */
  const_reverse_iterator crbegin() const noexcept;

  /*!
   * Returns a `const_reverse_iterator` pointing to the theoretical element
   * right before the first element in the map.
   */
  const_reverse_iterator crend() const noexcept;

  /*!
   * Returns a cursor pointing to the root node of the octree.
   *
   * Cursors provide an alternative way to navigate the tree backing the map.
   * While iterators visit the leaf nodes in turn a cursor allows intermediate
   * nodes to be visited by providing `to_parent()` and `to_child(std::size_t)`
   * methods instead of the `operator++()` and `operator--()` methods provided
   * by iterators.
   */
  cursor root() noexcept;
  const_cursor root() const noexcept;

  /*!
   * Returns a `const_cursor` pointing to the root node of the octree.
   */
  const_cursor croot() const noexcept;


  /*!
   * Returns whether the map is empty (i.e. whether its size is `0`).  Since the
   * octree's size will be at least `1` this method will always return
   * `false`.
   *
   * \return `false`.
   */
  bool empty() const noexcept;

  /*!
   * Returns the number of elements in the map.  Since the octree will always
   * have a root node (corresponding to the bounds of the entire map) this
   * method will always return a size greater than or equal to `1`.
   *
   * \return the number of elements in the map (greater than or equal to `1`).
   */
  size_type size() const noexcept;

  /*!
   * Returns the maximum number of elements this map can hold.
   *
   * This is the maximum potential size the container can reach due to known
   * system or library implementation limitations, but the container is by no
   * means guaranteed to be able to reach that size: it can still fail to
   * allocate storage at any point before that size is reached.
   *
   * \return the maximum number of elements a map can hold as content.
   */
  size_type max_size() const noexcept;


  /*!
   * If `p` is contained within the bounds of the map, this method returns a
   * reference to its mapped value.  If `p` is not contained within the bounds
   * of the map, this method throws a `std::out_of_range` exception.  This is a
   * change to the behaviour of `std::map` due to the different way values are
   * inserted into the octree.
   *
   * \param p  the point to look up in the map
   *
   * \return the mapped value for the bounds containing point `p`.
   */
  mapped_type & operator[](const point_type &);
  const mapped_type & operator[](const point_type &) const;
  mapped_type & at(const point_type &);
  const mapped_type & at(const point_type &) const;


  /*!
   * Divides the bounds referenced by the cursor or iterator around the point
   * defined by `centre` and attaches the nodes as children of the current node.
   * This increases the map's size by `7`.  Child nodes inherit the parent's
   * value.  After calling this method the node referenced by the cursor is
   * guaranteed not to be a leaf node.  Calling this method on a non-leaf node
   * has no effect.
   *
   * \param c  a cursor or iterator referencing the node to divide.
   */
  void divide(cursor &);
  void divide(iterator &);

  /*!
   * Collapses all child nodes of the node referenced by the cursor or iterator.
   * This decreases the map's size by at least `7`.  After calling this method
   * the node referenced by the cursor is guaranteed to be a leaf node.  Calling
   * this method on a leaf node has no effect.
   *
   * \param c  a cursor or iterator referencing the node to collapse.
   */
  void collapse(cursor &);
  void collapse(iterator &);

  /*!
   * Exchanges the content of the container by the content of `x`, which is
   * another map of the same type.
   *
   * \param x  another map of the same type as this whose content is swapped
   *             with that of this map.
   */
  void swap(map &);

  /*!
   * Removes all children of the root node, leaving this map with a size of `1`.
   */
  void clear() noexcept;

  /*!
   * Searches the map for a node containing the point `p` and returns an
   * iterator to it if found, otherwise it returns an iterator to `map::end`.
   *
   * \param p  the point to search for
   *
   * \return an iterator to the node containing point `p`, or `map::end` if the
   *           point is not within the bounds of the map.
   */
  iterator find(const point_type &);
  const_iterator find(const point_type &) const;

  /*!
   * Searches the map for nodes containing point `p` and returns the number of
   * matches.  Since `p` can only be contained in one node if it within the
   * bounds of the map, this method can only return `1` or `0`.
   *
   * \param p  the point to search for
   *
   * \return `1` if the point is within the bounds of the map, `0` otherwise.
   */
  size_type count(const point_type &) const;


  /*!
   * Returns a copy of the allocator associated with the map.
   *
   * \return the allocator.
   */
  allocator_type get_allocator() const noexcept;
};

/*!
 * The contents of map `x` are exchanged with those of `y`. Both map objects
 * must be of the same type, although sizes may differ.
 *
 * This is an overload of the generic algorithm `std::swap` that improves its
 * performance by mutually transferring ownership over their assets to the other
 * container (i.e., the containers exchange references to their data, without
 * actually performing any element copy or movement): It behaves as if x.swap(y)
 * was called.
 *
 * \param x  map containers of the same type
 * \param y  map containers of the same type
 */
template <class Vector3D, class Bounds3D, class T, class Contains, class Centre,
          class Alloc>
void swap(map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &,
          map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &);

/*!
 * Compares two maps for size and, if equal, compares each element sequentially
 * for equality (using `operator==`), stopping when a mismatch is found (as if
 * using algorithm `std::equal`).
 *
 * \param lhs  the map on the left hand side of the comparison
 * \param rhs  the map on the right hand side of the comparison
 *
 * \return `true` if the sizes and elements of `lhs` and `rhs` compare equal
 *            (using `operator==`), `false` otherwise.
 */
template <class Vector3D, class Bounds3D, class T, class Contains, class Centre,
          class Alloc>
bool operator==(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &,
                const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &);

/*!
 * Compares two maps for inequality.  Equivalent to the expression
 * `!(lhs == rhs)`.
 *
 * \param lhs  the map on the left hand side of the comparison
 * \param rhs  the map on the right hand side of the comparison
 *
 * \return `true` if the sizes or elements of `lhs` are not equal to `rhs`,
 *           `false` otherwise.
 */
template <class Vector3D, class Bounds3D, class T, class Contains, class Centre,
          class Alloc>
bool operator!=(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &,
                const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &);

/*!
 * Behaves equivalently to the algorithm `lexicographical_compare` using
 * `operator<` on each element and stopping at it's first occurrence.
 *
 * \param lhs  the map on the left hand side of the comparison
 * \param rhs  the map on the right hand side of the comparison
 *
 * \return `true` if `lhs` compares as less than `rhs`, `false` otherwise.
 */
template <class Vector3D, class Bounds3D, class T, class Contains, class Centre,
          class Alloc>
bool operator<(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &,
               const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &);

/*!
 * Equivalent to the expression `!(rhs < lhs)`.
 *
 * \param lhs  the map on the left hand side of the comparison
 * \param rhs  the map on the right hand side of the comparison
 *
 * \return `true` if `lhs` compares as less than or equal to `rhs`, `false`
 *           otherwise.
 */
template <class Vector3D, class Bounds3D, class T, class Contains, class Centre,
          class Alloc>
bool operator<=(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &,
                const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &);

/*!
 * Equivalent to the expression `rhs < lhs`.
 *
 * \param lhs  the map on the left hand side of the comparison
 * \param rhs  the map on the right hand side of the comparison
 *
 * \return `true` if `lhs` compares as greater than to `rhs`, `false` otherwise.
 */
template <class Vector3D, class Bounds3D, class T, class Contains, class Centre,
          class Alloc>
bool operator>(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &,
               const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &);

/*!
 * Equivalent to the expression `!(lhs < rhs)`.
 *
 * \param lhs  the map on the left hand side of the comparison
 * \param rhs  the map on the right hand side of the comparison
 *
 * \return `true` if `lhs` compares as greater than or equal to `rhs`, `false`
 *           otherwise.
 */
template <class Vector3D, class Bounds3D, class T, class Contains, class Centre,
          class Alloc>
bool operator>=(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &,
                const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> &);

}       // namespace octree

#endif

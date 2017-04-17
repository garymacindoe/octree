#ifndef OCTREE_MAP_H
#define OCTREE_MAP_H

#include <memory>
#include <utility>
#include <iterator>
#include <type_traits>
#include <stack>
#include <stdexcept>
#include <algorithm>

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

  /*!
   * Inner class representing a node in the octree.
   *
   * \param Value the type of the values stored in the octree.
   */
  template <class Value>
  struct octree_node {

    /*! Pointers to parent node and children. */
    octree_node * parent, * children[8];

    /*! The value stored in this node. */
    Value value;

  };

  template <class Value>
  void swap(octree_node<Value> & x, octree_node<Value> & y) {
    using std::swap;
    swap(x.value, y.value);
    swap(x.parent, y.parent);
    for (int i = 0; i != 8; ++i)
      swap(x.children[i], y.children[i]);
  }

  template <class Value, class Pointer, class Reference>
  class leaf_iterator :
      public std::iterator<std::bidirectional_iterator_tag, Value,
                           std::ptrdiff_t, Pointer, Reference> {
  public:
    leaf_iterator(octree_node<Value> * current = nullptr,
                  const std::stack<std::size_t> & path =
                           std::stack<std::size_t>()) :
        _current(current), _path(path) {}

    Reference operator*() const {
      return _current->value;
    }

    Pointer operator->() const {
      return &_current->value;
    }

    leaf_iterator & operator++() {
      _current = _current->parent;
      while (_current != nullptr && _path.top() == 7) {
        _path.pop();
        _current = _current->parent;
      }
      if (_current != nullptr) {
        _current = _current->children[++_path.top()];
        while (_current->children[0] != nullptr) {
          _path.push(0);
          _current = _current->children[0];
        }
      }
      return *this;
    }

    leaf_iterator & operator--() {
      _current = _current->parent;
      while (_current != nullptr && _path.top() == 0) {
        _path.pop();
        _current = _current->parent;
      }
      if (_current != nullptr) {
        _current = _current->children[--_path.top()];
        while (_current->children[7] != nullptr) {
          _path.push(7);
          _current = _current->children[7];
        }
      }
      return *this;
    }

    leaf_iterator & operator++(int) {
      leaf_iterator copy(*this);
      operator++();
      return copy;
    }

    leaf_iterator & operator--(int) {
      leaf_iterator copy(*this);
      operator--();
      return copy;
    }

    friend bool operator==(const leaf_iterator & lhs,
                           const leaf_iterator & rhs) {
      return lhs._current == rhs._current && lhs._path == rhs._path;
    }

    friend bool operator!=(const leaf_iterator & lhs,
                           const leaf_iterator & rhs) {
      return !(lhs == rhs);
    }

  private:

    /*!
     * Friends with map so that divide() and collapse() can access the node
     * referenced.
     */
    friend class map;

    /*! The current node (possibly null) */
    octree_node<Value> * _current;

    /*!
     * The indices of the child nodes visited on the path from the root node to
     * the current node.
     */
    std::stack<std::size_t> _path;

  };

  template <class Value, class Pointer, class Reference>
  class octree_cursor {
  public:
    typedef Value value_type;
    typedef Pointer pointer;
    typedef Reference reference;

    octree_cursor(octree_node<Value> * root) : _current(root) {}

    reference operator*() const {
      return _current->value;
    }

    pointer operator->() const {
      return &_current->value;
    }

    void to_parent() {
      _current = _current->parent;
    }

    void to_child(std::size_t i) {
      if (i >= 8)
        throw std::out_of_range("octree::map::cursor::to_child(): invalid "
                                "child index >= 8");
      _current = _current->children[i];
    }

    bool is_root() const {
      return _current->parent == nullptr;
    }

    bool is_leaf() const {
      return _current->children[0] == nullptr;
    }

    friend bool operator==(const octree_cursor & lhs,
                           const octree_cursor & rhs) {
      return lhs._current == rhs._current;
    }

    friend bool operator!=(const octree_cursor & lhs,
                           const octree_cursor & rhs) {
      return !(lhs == rhs);
    }

  private:

    /*!
     * Friends with map so that divide() and collapse() can access the node
     * referenced.
     */
    friend class map;

    /*! The current node (possibly null) */
    octree_node<Value> * _current;

  };

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
  typedef leaf_iterator<value_type, pointer, reference> iterator;

  /*! A bidirectional iterator to `const value_type` */
  typedef leaf_iterator<value_type, const_pointer,
                        const_reference> const_iterator;

  /*! A tree cursor to `value_type` */
  typedef octree_cursor<value_type, pointer, reference> cursor;

  /*! A tree cursor to `const value_type` */
  typedef octree_cursor<value_type, const_pointer,
                        const_reference> const_cursor;

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
  explicit map(const volume_type & bounds = volume_type(),
               const mapped_type & value = mapped_type(),
               const volume_contains & contains = volume_contains(),
               const volume_centre & centre = volume_centre(),
               const allocator_type & alloc = allocator_type()) :
      _n(1), _contains(contains), _centre(centre), _allocator(alloc) {
    _allocator.construct(std::addressof(_root.value), bounds, value);
    _root.parent = nullptr;
    for (int i = 0; i != 8; ++i)
      _root.children[i] = nullptr;
  }

  /*!
   * Constructs a map with a copy of each of the elements in `x`.
   *
   * \param x      another map object of the same type, whose contents are
   *                 copied
   * \param alloc  Allocator object.  The container keeps and uses an internal
   *                 copy of this object.
   */
  map(const map & x) :
    _n(x._n), _contains(x._contains), _centre(x._centre),
    _allocator(std::allocator_traits<allocator_type>::
               select_on_container_copy_construction(x._allocator)) {
    _allocator.construct(std::addressof(_root.value), x._root.value);
    _root.parent = nullptr;
    for (int i = 0; i != 8; ++i)
      _root.children[i] = copy(x._root.children[i], &_root);
  }
  map(const map & x, const allocator_type & alloc) :
    _n(x._n), _contains(x._contains), _centre(x._centre),
    _allocator(alloc) {
    _allocator.construct(std::addressof(_root.value), x._root.value);
    _root.parent = nullptr;
    for (int i = 0; i != 8; ++i)
      _root.children[i] = copy(x._root.children[i], &_root);
  }

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
  map(map && x) :
    _n(x._n), _contains(std::move(x._contains)),
    _centre(std::move(x._contains)), _allocator(std::move(x._allocator)) {
    _allocator.construct(std::addressof(_root.value), std::move(x._root.value));
    _root.parent = nullptr;
    for (int i = 0; i != 8; ++i) {
      _root.children[i] = x._root.children[i];
      _root.children[i].parent = &_root;
      x._root.children[i] = nullptr;
    }
  }
  map(map && x, const allocator_type & alloc) :
    _n(x._n), _contains(std::move(x._contains)),
    _centre(std::move(x._contains)), _allocator(alloc) {
    _allocator.construct(std::addressof(_root.value), std::move(x._root.value));
    _root.parent = nullptr;
    if (_allocator == x._allocator) {
      for (int i = 0; i != 8; ++i) {
        _root.children[i] = x._root.children[i];
        _root.children[i].parent = &_root;
        x._root.children[i] = nullptr;
      }
    }
    else {
      for (int i = 0; i != 8; ++i)
        _root.children[i] = copy(x._root.children[i], &_root);
    }
  }

  /*!
   * Copies all the elements of `x` into this map.
   *
   * \param x  another map object of the same type, whose contents are copied
   */
  map & operator=(const map & x) {
    if (*this != x) {
      collapse(&_root);
      _allocator.destroy(std::addressof(_root.value));
      _n = x._n;
      _contains = x._contains;
      _centre = x._centre;
      _allocator = x._allocator;
      _allocator.construct(std::addressof(_root.value), x._root.value);
      for (int i = 0; i != 8; ++i)
        _root.children[i] = copy(x._root.children[i], &_root);
    }
    return *this;
  }

  /*!
   * Moves the elements of `x` into this map.  `x` is left in an unspecified but
   * valid state.
   *
   * \param x  another map object of the same type, whose contents are moved
   *             into this map
   */
  map & operator=(map && x) {
    if (*this != x) {
      collapse(&_root);
      _n = x._n;
      _contains = std::move(x._contains);
      _centre = std::move(x._centre);
      _allocator = std::move(x._allocator);
      _root.value = std::move(x._root.value);
      for (int i = 0; i != 8; ++i) {
        _root.children[i] = x._root.children[i];
        _root.children[i].parent = &root;
        x._root.children[i] = nullptr;
      }
    }
    return *this;
  }

  /*!
   * Destroys the octree map.
   */
  ~map() {
    collapse(&_root);
  }


  /*!
   * Returns an iterator pointing to the first element in the map.
   */
  iterator begin() noexcept {
    std::stack<std::size_t> path;
    octree_node<value_type> * node = first(&_root, path);
    return iterator(node, path);
  }
  const_iterator begin() const noexcept {
    std::stack<std::size_t> path;
    octree_node<value_type> * node = first(&_root, path);
    return const_iterator(node, path);
  }

  /*!
   * Returns an iterator pointing to one-past-the-end element in the map.
   *
   * The value returned shall not be dereferenced.
   */
  iterator end() noexcept {
    return iterator();
  }
  const_iterator end() const noexcept {
    return const_iterator();
  }

  /*!
   * Returns a reverse iterator pointing to the last element in the map.
   */
  reverse_iterator rbegin() noexcept {
    std::stack<std::size_t> path;
    octree_node<value_type> * node = last(&_root, path);
    return reverse_iterator(node, path);
  }
  const_reverse_iterator rbegin() const noexcept {
    std::stack<std::size_t> path;
    octree_node<value_type> * node = last(&_root, path);
    return const_reverse_iterator(node, path);
  }

  /*!
   * Returns a reverse iterator pointing to the theoretical element right before
   * the first element in the map.
   */
  reverse_iterator rend() noexcept {
    return reverse_iterator();
  }
  const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator();
  }

  /*!
   * Returns a `const_iterator` pointing to the first element in the map.
   */
  const_iterator cbegin() const noexcept {
    std::stack<std::size_t> path;
    octree_node<value_type> * node = first(&_root, path);
    return const_iterator(node, path);
  }

  /*!
   * Returns a `const_iterator` pointing to one-past-the-end element in the map.
   *
   * The value returned shall not be dereferenced.
   */
  const_iterator cend() const noexcept {
    return const_iterator();
  }

  /*!
   * Returns a `const_reverse_iterator` pointing to the last element in the map.
   */
  const_reverse_iterator crbegin() const noexcept {
    std::stack<std::size_t> path;
    octree_node<value_type> * node = last(&_root, path);
    return const_reverse_iterator(node, path);
  }

  /*!
   * Returns a `const_reverse_iterator` pointing to the theoretical element
   * right before the first element in the map.
   */
  const_reverse_iterator crend() const noexcept {
    return const_reverse_iterator();
  }

  /*!
   * Returns a cursor pointing to the root node of the octree.
   *
   * Cursors provide an alternative way to navigate the tree backing the map.
   * While iterators visit the leaf nodes in turn a cursor allows intermediate
   * nodes to be visited by providing `to_parent()` and `to_child(std::size_t)`
   * methods instead of the `operator++()` and `operator--()` methods provided
   * by iterators.
   */
  cursor root() noexcept {
    return cursor(&_root);
  }
  const_cursor root() const noexcept {
    return const_cursor(&_root);
  }

  /*!
   * Returns a `const_cursor` pointing to the root node of the octree.
   */
  const_cursor croot() const noexcept {
    return const_cursor(&_root);
  }


  /*!
   * Returns whether the map is empty (i.e. whether its size is `0`).  Since the
   * octree's size will be at least `1` this method will always return
   * `false`.
   *
   * \return `false`.
   */
  bool empty() const noexcept {
    return false;
  }

  /*!
   * Returns the number of elements in the map.  Since the octree will always
   * have a root node (corresponding to the bounds of the entire map) this
   * method will always return a size greater than or equal to `1`.
   *
   * \return the number of elements in the map (greater than or equal to `1`).
   */
  size_type size() const noexcept {
    return _n;
  }

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
  size_type max_size() const noexcept {
    return std::allocator_traits<allocator_type>::max_size(_allocator);
  }


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
  mapped_type & operator[](const point_type & p) {
    octree_node<value_type> * node = locate(p);
    if (node == nullptr)
      throw std::out_of_range("octree::map::operator[]: point out of range");
    return node->second;
  }
  const mapped_type & operator[](const point_type & p) const {
    octree_node<value_type> * node = locate(p);
    if (node == nullptr)
      throw std::out_of_range("octree::map::operator[]: point out of range");
    return node->second;
  }
  mapped_type & at(const point_type & p) {
    octree_node<value_type> * node = locate(p);
    if (node == nullptr)
      throw std::out_of_range("octree::map::at(): point out of range");
    return node->second;
  }
  const mapped_type & at(const point_type & p) const  {
    octree_node<value_type> * node = locate(p);
    if (node == nullptr)
      throw std::out_of_range("octree::map::at(): point out of range");
    return node->second;
  }


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
  void divide(cursor & c) {
    divide(c._current);
  }
  void divide(iterator & i) {
    divide(i._current);
  }

  /*!
   * Collapses all child nodes of the node referenced by the cursor or iterator.
   * This decreases the map's size by at least `7`.  After calling this method
   * the node referenced by the cursor is guaranteed to be a leaf node.  Calling
   * this method on a leaf node has no effect.
   *
   * \param c  a cursor or iterator referencing the node to collapse.
   */
  void collapse(cursor & c) {
    collapse(c._current);
  }
  void collapse(iterator & i) {
    collapse(i._current);
  }

  /*!
   * Exchanges the content of the container by the content of `x`, which is
   * another map of the same type.
   *
   * \param x  another map of the same type as this whose content is swapped
   *             with that of this map.
   */
  void swap(map & x) {
    using std::swap;
    swap(_root, x._root);
    swap(_n, x._n);
    swap(_contains, x._contains);
    swap(_centre, x._centre);
    swap(_allocator, x._allocator);
  }

  /*!
   * Removes all children of the root node, leaving this map with a size of `1`.
   */
  void clear() noexcept {
    collapse(&_root);
  }

  /*!
   * Searches the map for a node containing the point `p` and returns an
   * iterator to it if found, otherwise it returns an iterator to `map::end`.
   *
   * \param p  the point to search for
   *
   * \return an iterator to the node containing point `p`, or `map::end` if the
   *           point is not within the bounds of the map.
   */
  iterator find(const point_type & p) {
    std::stack<std::size_t> path;
    octree_node<value_type> * node = locate(p, path);
    return iterator(node, path);
  }
  const_iterator find(const point_type & p) const {
    std::stack<std::size_t> path;
    octree_node<value_type> * node = locate(p, path);
    return const_iterator(node, path);
  }

  /*!
   * Searches the map for nodes containing point `p` and returns the number of
   * matches.  Since `p` can only be contained in one node if it within the
   * bounds of the map, this method can only return `1` or `0`.
   *
   * \param p  the point to search for
   *
   * \return `1` if the point is within the bounds of the map, `0` otherwise.
   */
  size_type count(const point_type & p) const {
    return (_contains(_root.value.first, p)) ? 1 : 0;
  }


  /*!
   * Returns a copy of the allocator associated with the map.
   *
   * \return the allocator.
   */
  allocator_type get_allocator() const noexcept {
    return _allocator;
  }

  /*!
   * Returns a copy of the function object used to calculate whether a point is
   * within a volume.
   *
   * \return the function object
   */
  volume_contains volume_cont() const {
    return _contains;
  }

  /*!
   * Returns a copy of the function objects used to calculate the centre of a
   * volume.
   *
   * \return the function object
   */
  volume_centre volume_cent() const {
    return _centre;
  }

private:

  typedef std::allocator_traits<allocator_type> allocator_traits;

  typedef typename allocator_traits::template
      rebind_alloc<octree_node<value_type>> node_allocator;

  typedef std::allocator_traits<node_allocator> node_allocator_traits;

  static octree_node<value_type> * first(octree_node<value_type> * node,
                                         std::stack<std::size_t> & path) {
    while (node->children[0] != nullptr) {
      node = node->children[0];
      path.push(0);
    }
    return node;
  }

  static octree_node<value_type> * last(octree_node<value_type> * node,
                                        std::stack<std::size_t> & path) {
    while (node->children[7] != nullptr) {
      node = node->children[7];
      path.push(7);
    }
    return node;
  }
  octree_node<value_type> * copy(const octree_node<value_type> * node,
                                 octree_node<value_type> * parent) {
    if (node == nullptr)
      return nullptr;

    // Create an allocator for nodes
    node_allocator allocator(_allocator);

    octree_node<value_type> * res = nullptr;
    try {
      // Allocate the node using the node allocator
      res = node_allocator_traits::allocate(allocator, 1);
      // Construct the value in the node using the value allocator
      allocator_traits::construct(_allocator, std::addressof(res->value),
                                  node->value);
      // Set the node's parent
      res->parent = parent;
    } catch (...) {
      // Deallocate the node
      if (res != nullptr)
        node_allocator_traits::deallocate(allocator, res, 1);
      // Rethrow the original exception
      throw;
    }

    // Copy the children recursively setting their parent to the new node
    for (int i = 0; i != 8; ++i) {
      try {
        res->children[i] = copy(node->children[i], res);
      } catch (...) {
        // Destroy the children created so far
        for (int j = 0; j != i; ++j) {
          allocator_traits::destroy(_allocator,
                                    std::addressof(res->children[j]->value));
          node_allocator_traits::deallocate(allocator, res->children[j], 1);
        }
        // Destroy and deallocate the node
        allocator_traits::destroy(_allocator, std::addressof(res->value));
        node_allocator_traits::deallocate(allocator, res, 1);
        // Rethrow the original exception
        throw;
      }
    }

    return res;
  }

  template <class _T>
  struct _null_stack {
    void push(const T &) {}
  };

  template <class stack = _null_stack<std::size_t>>
  octree_node<value_type> * locate(const point_type & point,
                                   stack & path = stack()) const {
    // If the point is not contained in the bounds of the root node it's not
    // going to be in any of the sub-nodes either
    if (!_contains(_root.value.first, point))
      return nullptr;

    // While node is not a leaf (if the first child is null they're all null)
    octree_node<value_type> * node = &_root;
    while (node->children[0] != nullptr) {
      // Find centre of node
      const Vector3D centre = _centre(node->value.first);

      // Work out the index of the octant in which the point lies
      std::size_t i = 0;
      if (point.x() >= centre.x())
        i |= 1;
      if (point.y() >= centre.y())
        i |= 1 << 1;
      if (point.z() >= centre.z())
        i |= 1 << 2;

      // Go to the correct child octant
      node = node->children[i];

      // Update the path taken
      path.push(i);
    }

    // Return the (leaf) node containing the point
    return node;
  }

  typedef std::allocator_traits<allocator_type> allocator_traits;

  typedef typename allocator_traits::template
      rebind_alloc<octree_node<value_type>> node_allocator;

  typedef std::allocator_traits<node_allocator> node_allocator_traits;

  void divide(octree_node<value_type> * node) {
    // Divide the parent bounds around the centre to calculate the bounds of the
    // children
    const volume_type parent = node->value.first;
    const point_type centre = _centre(parent);

    // Calculate the bounds in left-right, down-up, back-front order
    const volume_type bounds[] = {
      { parent.xmin(), centre.x(), parent.ymin(), centre.y(), parent.zmin(), centre.z() },
      { centre.x(), parent.xmax(), parent.ymin(), centre.y(), parent.zmin(), centre.z() },
      { parent.xmin(), centre.x(), centre.y(), parent.ymax(), parent.zmin(), centre.z() },
      { centre.x(), parent.xmax(), centre.y(), parent.ymax(), parent.zmin(), centre.z() },
      { parent.xmin(), centre.x(), parent.ymin(), centre.y(), centre.z(), parent.zmax() },
      { centre.x(), parent.xmax(), parent.ymin(), centre.y(), centre.z(), parent.zmax() },
      { parent.xmin(), centre.x(), centre.y(), parent.ymax(), centre.z(), parent.zmax() },
      { centre.x(), parent.xmax(), centre.y(), parent.ymax(), centre.z(), parent.zmax() }
    };

    // Create an allocator for nodes
    node_allocator allocator(_allocator);
    for (int i = 0; i != 8; ++i) {
      if (node->children[i] == nullptr) {
        try {
          // Allocate a child node using the node allocator
          node->children[i] = node_allocator_traits::allocate(allocator, 1);
          // Construct the value in the node using the value allocator
          allocator_traits::construct(_allocator,
                                      std::addressof(node->children[i]->value),
                                      bounds[i], node->value.second);
          // Set the child node's parent and children
          node->children[i]->parent = node;
          for (int j = 0; j != 8; ++j)
            node->children[i]->children[j] = nullptr;
        } catch (...) {
          // If any allocation/constructor threw destroy and deallocate all the
          // children created so far
          for (int j = 0; j != i; ++j) {
            // Destroy the node value
            allocator_traits::destroy(_allocator,
                                      std::addressof(node->children[j]->value));
            node_allocator_traits::deallocate(allocator, node->children[j], 1);
            // Reset the parent's child to null
            node->children[j] = nullptr;
          }
          // Destroy and deallocate the node that was being created when the
          // value constructor threw
          if (node->children[i] != nullptr)
            node_allocator_traits::deallocate(allocator, node->children[i], 1);
          // Rethrow the original exception
          throw;
        }
      }
    }
    _n += 7;
  }

  void collapse(octree_node<value_type> * node) {
    // Create an allocator for nodes
    node_allocator allocator(_allocator);
    for (int i = 0; i != 8; ++i) {
      if (node->children[i] != nullptr) {
        collapse(node->children[i]);
        // Destroy each node value and node, then deallocate the node
        node_allocator_traits::destroy(allocator,
                                       std::addressof(node->children[i]->value));
        node_allocator_traits::deallocate(allocator, node->children[i], 1);
        // Reset child to null
        node->children[i] = nullptr;
      }
    }
    _n -= 7;
  }

  octree_node<value_type> * copy(const octree_node<value_type> * node,
                                 octree_node<value_type> * parent) {
    if (node == nullptr)
      return nullptr;

    // Create an allocator for nodes
    node_allocator allocator(_allocator);

    octree_node<value_type> * res = nullptr;
    try {
      // Allocate the node using the node allocator
      res = node_allocator_traits::allocate(allocator, 1);
      // Construct the value in the node using the value allocator
      allocator_traits::construct(_allocator, std::addressof(res->value),
                                  node->value);
      // Set the node's parent
      res->parent = parent;
    } catch (...) {
      // Deallocate the node
      if (res != nullptr)
        node_allocator_traits::deallocate(allocator, res, 1);
      // Rethrow the original exception
      throw;
    }

    // Copy the children recursively setting their parent to the new node
    for (int i = 0; i != 8; ++i) {
      try {
        res->children[i] = copy(node->children[i], res);
      } catch (...) {
        // Destroy the children created so far
        for (int j = 0; j != i; ++j) {
          allocator_traits::destroy(_allocator,
                                    std::addressof(res->children[j]->value));
          node_allocator_traits::deallocate(allocator, res->children[j], 1);
        }
        // Destroy and deallocate the node
        allocator_traits::destroy(_allocator, std::addressof(res->value));
        node_allocator_traits::deallocate(allocator, res, 1);
        // Rethrow the original exception
        throw;
      }
    }

    return res;
  }

  octree_node<value_type> * locate(const point_type & point,
                                   std::stack<std::size_t> & path) const {
    // If the point is not contained in the bounds of the root node it's not
    // going to be in any of the sub-nodes either
    if (!_contains(_root.value.first, point))
      return nullptr;

    // While node is not a leaf (if the first child is null they're all null)
    octree_node<value_type> * node = &_root;
    while (node->children[0] != nullptr) {
      // Find centre of node
      const Vector3D centre = _centre(node->value.first);

      // Work out the index of the octant in which the point lies
      std::size_t i = 0;
      if (point.x() >= centre.x())
        i |= 1;
      if (point.y() >= centre.y())
        i |= 1 << 1;
      if (point.z() >= centre.z())
        i |= 1 << 2;

      // Go to the correct child octant
      node = node->children[i];

      // Update the path taken
      path.push(i);
    }

    // Return the (leaf) node containing the point
    return node;
  }

  octree_node<value_type> * locate(const point_type & point) const {
    // If the point is not contained in the bounds of the root node it's not
    // going to be in any of the sub-nodes either
    if (!_contains(_root.value.first, point))
      return nullptr;

    // While node is not a leaf (if the first child is null they're all null)
    octree_node<value_type> * node = &_root;
    while (node->children[0] != nullptr) {
      // Find centre of node
      const Vector3D centre = _centre(node->value.first);

      // Work out the index of the octant in which the point lies
      std::size_t i = 0;
      if (point.x() >= centre.x())
        i |= 1;
      if (point.y() >= centre.y())
        i |= 1 << 1;
      if (point.z() >= centre.z())
        i |= 1 << 2;

      // Go to the correct child octant
      node = node->children[i];
    }

    // Return the (leaf) node containing the point
    return node;
  }

  void divide(octree_node<value_type> * node) {
    // Divide the parent bounds around the centre to calculate the bounds of the
    // children
    const volume_type parent = node->value.first;
    const point_type centre = _centre(parent);

    // Calculate the bounds in left-right, down-up, back-front order
    const volume_type bounds[] = {
      { parent.xmin(), centre.x(), parent.ymin(), centre.y(), parent.zmin(), centre.z() },
      { centre.x(), parent.xmax(), parent.ymin(), centre.y(), parent.zmin(), centre.z() },
      { parent.xmin(), centre.x(), centre.y(), parent.ymax(), parent.zmin(), centre.z() },
      { centre.x(), parent.xmax(), centre.y(), parent.ymax(), parent.zmin(), centre.z() },
      { parent.xmin(), centre.x(), parent.ymin(), centre.y(), centre.z(), parent.zmax() },
      { centre.x(), parent.xmax(), parent.ymin(), centre.y(), centre.z(), parent.zmax() },
      { parent.xmin(), centre.x(), centre.y(), parent.ymax(), centre.z(), parent.zmax() },
      { centre.x(), parent.xmax(), centre.y(), parent.ymax(), centre.z(), parent.zmax() }
    };

    // Create an allocator for nodes
    node_allocator allocator(_allocator);
    for (int i = 0; i != 8; ++i) {
      if (node->children[i] == nullptr) {
        try {
          // Allocate a child node using the node allocator
          node->children[i] = node_allocator_traits::allocate(allocator, 1);
          // Construct the value in the node using the value allocator
          allocator_traits::construct(_allocator,
                                      std::addressof(node->children[i]->value),
                                      bounds[i], node->value.second);
          // Set the child node's parent and children
          node->children[i]->parent = node;
          for (int j = 0; j != 8; ++j)
            node->children[i]->children[j] = nullptr;
        } catch (...) {
          // If any allocation/constructor threw destroy and deallocate all the
          // children created so far
          for (int j = 0; j != i; ++j) {
            // Destroy the node value
            allocator_traits::destroy(_allocator,
                                      std::addressof(node->children[j]->value));
            node_allocator_traits::deallocate(allocator, node->children[j], 1);
            // Reset the parent's child to null
            node->children[j] = nullptr;
          }
          // Destroy and deallocate the node that was being created when the
          // value constructor threw
          if (node->children[i] != nullptr)
            node_allocator_traits::deallocate(allocator, node->children[i], 1);
          // Rethrow the original exception
          throw;
        }
      }
    }
    _n += 7;
  }

  void collapse(octree_node<value_type> * node) {
    // Create an allocator for nodes
    node_allocator allocator(_allocator);
    for (int i = 0; i != 8; ++i) {
      if (node->children[i] != nullptr) {
        collapse(node->children[i]);
        // Destroy each node value and node, then deallocate the node
        node_allocator_traits::destroy(allocator,
                                       std::addressof(node->children[i]->value));
        node_allocator_traits::deallocate(allocator, node->children[i], 1);
        // Reset child to null
        node->children[i] = nullptr;
      }
    }
    _n -= 7;
  }

  /*!
   * The root node (not a pointer since it will never be null).
   */
  octree_node<value_type> _root;

  /*!
   * The number of leaf nodes in the tree.
   * Always greater than or equal to `1`.
   */
  std::size_t _n;

  /*!
   * Functor that tests whether a point is contained within a volume.
   */
  volume_contains _contains;

  /*!
   * Functor that calculates the centre of a volume.
   */
  volume_centre _centre;

  /*!
   * Memory allocator.
   */
  allocator_type _allocator;

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
void swap(map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & x,
          map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & y) {
  x.swap(y);
}

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
bool operator==(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & lhs,
                const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & rhs) {
  return lhs.size() == rhs.size() &&
         std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin());
}

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
bool operator!=(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & lhs,
                const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & rhs) {
  return !(lhs == rhs);
}

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
bool operator<(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & lhs,
               const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & rhs) {
  return std::lexicographical_compare(lhs.cbegin(), lhs.cend(),
                                      rhs.cbegin(), rhs.cend());
}

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
bool operator<=(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & lhs,
                const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & rhs) {
  return !(rhs < lhs);
}

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
bool operator>(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & lhs,
               const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & rhs) {
  return rhs < lhs;
}

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
bool operator>=(const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & lhs,
                const map<Vector3D, Bounds3D, T, Contains, Centre, Alloc> & rhs) {
  return !(lhs < rhs);
}

}       // namespace octree

#endif

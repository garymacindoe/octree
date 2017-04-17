#include "vector3D.hpp"
#include "bounds3D.hpp"
#include "map.hpp"

#include <cassert>

#if __cplusplus == 201103L
#define STATIC_ASSERT(...) static_assert(__VA_ARGS__, #__VA_ARGS__)
#else
#define STATIC_ASSERT(...) static_assert(__VA_ARGS__)
#endif

int main() {
  {
    STATIC_ASSERT(std::is_same<octree::contains<octree::bounds3D<double>,
                               octree::vector3D<double>>::volume_type,
                               octree::bounds3D<double>>::value);
    STATIC_ASSERT(std::is_same<octree::contains<octree::bounds3D<double>,
                               octree::vector3D<double>>::point_type,
                               octree::vector3D<double>>::value);
    const octree::contains<octree::bounds3D<double>,
                           octree::vector3D<double>> contains{};
    const octree::bounds3D<double> bounds(-1.0, 1.0, -2.0, 2.0, -3.0, 3.0);
    const octree::vector3D<double> point0( 1.0,  2.0,  3.0);
    const octree::vector3D<double> point1(-1.0,  2.0,  3.0);
    const octree::vector3D<double> point2( 1.0, -2.0,  3.0);
    const octree::vector3D<double> point3(-1.0, -2.0,  3.0);
    const octree::vector3D<double> point4( 1.0,  2.0, -3.0);
    const octree::vector3D<double> point5(-1.0,  2.0, -3.0);
    const octree::vector3D<double> point6( 1.0, -2.0, -3.0);
    const octree::vector3D<double> point7(-1.0, -2.0, -3.0);
    assert(!contains(bounds, point0));
    assert(!contains(bounds, point1));
    assert(!contains(bounds, point2));
    assert(!contains(bounds, point3));
    assert(!contains(bounds, point4));
    assert(!contains(bounds, point5));
    assert(!contains(bounds, point6));
    assert( contains(bounds, point7));
  }

  {
    STATIC_ASSERT(std::is_same<octree::centre<octree::bounds3D<double>,
                               octree::vector3D<double>>::volume_type,
                               octree::bounds3D<double>>::value);
    STATIC_ASSERT(std::is_same<octree::centre<octree::bounds3D<double>,
                               octree::vector3D<double>>::point_type,
                               octree::vector3D<double>>::value);
    const octree::centre<octree::bounds3D<double>,
                         octree::vector3D<double>> centre{};
    const octree::bounds3D<double> bounds(-3.0, -2.0, -1.0,  1.0,  2.0,  3.0);
    const octree::vector3D<double> expected(-2.5,  0.0,  2.5);
    const octree::vector3D<double> actual = centre(bounds);
    assert(actual.x() == expected.x());
    assert(actual.y() == expected.y());
    assert(actual.z() == expected.z());
  }

  return 0;
}

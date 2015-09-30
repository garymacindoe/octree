# Octree

Simple octree-backed map class.  Mostly follows the interface of `std::map`,
with the volumes being mapped to values and keys being points.  Rather than
inserting arbitrary key/value pairs into the map the map starts with an initial
volume which is then subdivided.  Values are stored in leaf nodes.

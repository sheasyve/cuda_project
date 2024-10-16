#ifndef MESH_CUH
#define MESH_CUH

#include <vector>
#include <optional>
#include <tuple>
#include "triangle.cuh"
#include "../utils/ray.cuh"

class Mesh {
public:
    std::vector<Triangle> triangles;
    Mesh(const std::vector<Triangle>& tris);
    std::optional<std::tuple<double, Eigen::Vector3d, const Triangle*>> intersects(const Ray& ray) const;
};

#endif
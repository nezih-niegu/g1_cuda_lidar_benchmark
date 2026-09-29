#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace g1bench {
struct Point { float x, y, z; };
// All backends retain the first input point from each occupied voxel.
// Coordinates must be inside [-1048576,1048575] voxel indices on all axes.
struct FilterParams { float min_range, max_range, voxel_size; };
std::vector<std::uint32_t> cuda_naive_indices(const std::vector<Point>&, FilterParams);
std::vector<std::uint32_t> cuda_optimized_indices(const std::vector<Point>&, FilterParams);
}

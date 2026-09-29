#include "g1_cuda_lidar_benchmark/filter.hpp"
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/unique.h>
#include <thrust/transform.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/execution_policy.h>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <unordered_set>
#include <cmath>
#include <limits>
namespace g1bench {
namespace {
void check(cudaError_t e, const char* operation) {
  if (e != cudaSuccess) throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(e));
}
constexpr unsigned long long BAD = 0xffffffffffffffffULL;
__host__ __device__ unsigned long long voxel_key(Point p, FilterParams f) {
  if (!(isfinite(p.x) && isfinite(p.y) && isfinite(p.z))) return BAD;
  double r2 = double(p.x)*p.x + double(p.y)*p.y + double(p.z)*p.z;
  if (r2 < double(f.min_range)*f.min_range || r2 > double(f.max_range)*f.max_range) return BAD;
  // Double precision floor on host and device to minimize boundary disagreements.
  long long x = (long long)floor(double(p.x)/double(f.voxel_size));
  long long y = (long long)floor(double(p.y)/double(f.voxel_size));
  long long z = (long long)floor(double(p.z)/double(f.voxel_size));
  if (x < -1048576LL || x > 1048575LL || y < -1048576LL || y > 1048575LL || z < -1048576LL || z > 1048575LL) return BAD;
  return ((unsigned long long)(x+1048576LL) << 42) |
         ((unsigned long long)(y+1048576LL) << 21) |
          (unsigned long long)(z+1048576LL);
}
__global__ void naive_key_kernel(const Point* pts, unsigned long long* keys, size_t n, FilterParams f) {
  size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) keys[i] = voxel_key(pts[i], f);
}
struct BuildKey {
  const Point* p;
  FilterParams f;
  __host__ __device__ unsigned long long operator()(std::uint32_t i) const {return voxel_key(p[i], f);}
};
}
// Deliberately naive GPU: allocate each frame, launch simple kernel, copy all keys
// to CPU, and complete the voxel deduplication on CPU. This is a real but poor GPU baseline.
std::vector<std::uint32_t> cuda_naive_indices(const std::vector<Point>& points, FilterParams f) {
  if (points.empty()) return {};
  Point* d_pts = nullptr;
  unsigned long long* d_keys = nullptr;
  check(cudaMalloc(&d_pts, points.size()*sizeof(Point)), "cudaMalloc points");
  try { check(cudaMalloc(&d_keys, points.size()*sizeof(unsigned long long)), "cudaMalloc keys"); }
  catch (...) { cudaFree(d_pts); throw; }
  try {
    check(cudaMemcpy(d_pts, points.data(), points.size()*sizeof(Point), cudaMemcpyHostToDevice), "H2D");
    int threads = 256;
    naive_key_kernel<<<(points.size()+threads-1)/threads, threads>>>(d_pts,d_keys,points.size(),f);
    check(cudaGetLastError(), "naive kernel launch");
    std::vector<unsigned long long> keys(points.size());
    check(cudaMemcpy(keys.data(), d_keys, keys.size()*sizeof(unsigned long long), cudaMemcpyDeviceToHost), "D2H");
    std::unordered_set<unsigned long long> seen;
    std::vector<std::uint32_t> selected;
    for(std::uint32_t i=0; i<keys.size(); ++i) if(keys[i]!=BAD && seen.insert(keys[i]).second) selected.push_back(i);
    cudaFree(d_pts); cudaFree(d_keys);
    return selected;
  } catch (...) { cudaFree(d_pts); cudaFree(d_keys); throw; }
}
// Optimized GPU: reuse device buffers, build keys, stable sort, unique occupied
// voxels on GPU and transfer only selected indices to the host. The stable sort
// ensures each voxel retains the first original input point, like the CPU.
std::vector<std::uint32_t> cuda_optimized_indices(const std::vector<Point>& points, FilterParams f) {
  if (points.empty()) return {};
  // Persistent per-thread allocation. Benchmark a single-threaded executor per node.
  thread_local thrust::device_vector<Point> d_pts;
  thread_local thrust::device_vector<unsigned long long> d_keys;
  thread_local thrust::device_vector<std::uint32_t> d_indices;
  const size_t n=points.size();
  if(d_pts.size()<n) { d_pts.resize(n); d_keys.resize(n); d_indices.resize(n); }
  check(cudaMemcpy(thrust::raw_pointer_cast(d_pts.data()),points.data(),n*sizeof(Point),cudaMemcpyHostToDevice),"optimized H2D");
  auto keys=d_keys.begin();
  auto ids=d_indices.begin();
  thrust::sequence(thrust::device,ids,ids+n);
  thrust::transform(thrust::device, thrust::make_counting_iterator<std::uint32_t>(0),
     thrust::make_counting_iterator<std::uint32_t>(static_cast<std::uint32_t>(n)),keys,
     BuildKey{thrust::raw_pointer_cast(d_pts.data()),f});
  thrust::stable_sort_by_key(thrust::device, keys, keys+n, ids);
  auto ends=thrust::unique_by_key(thrust::device, keys, keys+n, ids);
  size_t count=ends.first-keys;
  if(count) {
    unsigned long long last;
    check(cudaMemcpy(&last,thrust::raw_pointer_cast(d_keys.data())+count-1,sizeof(last),cudaMemcpyDeviceToHost),"last key");
    if(last==BAD) --count;
  }
  std::vector<std::uint32_t> result(count);
  if(count) check(cudaMemcpy(result.data(),thrust::raw_pointer_cast(d_indices.data()),count*sizeof(std::uint32_t),cudaMemcpyDeviceToHost),"indices D2H");
  std::sort(result.begin(),result.end()); // output in original point order, like CPU
  return result;
}
}

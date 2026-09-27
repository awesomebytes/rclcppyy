#ifndef RCLCPPYY_BENCHMARK_BOUNDARY_KERNEL_HPP
#define RCLCPPYY_BENCHMARK_BOUNDARY_KERNEL_HPP

#include <cstdint>
#include <functional>

namespace rclcppyy_boundary_benchmark {

using Transform = std::function<std::uint64_t(std::uint64_t, std::uint64_t)>;

inline std::uint64_t transform(std::uint64_t state, std::uint64_t index)
{
  state ^= index + 0x9e3779b97f4a7c15ULL + (state << 6U) + (state >> 2U);
  return state * 0xbf58476d1ce4e5b9ULL + 0x94d049bb133111ebULL;
}

inline std::uint64_t run_fused(std::uint64_t iterations, std::uint64_t seed)
{
  std::uint64_t state = seed;
  for (std::uint64_t index = 0; index < iterations; ++index) {
    state = transform(state, index);
  }
  return state;
}

inline std::uint64_t run_python_boundary(
  std::uint64_t iterations,
  std::uint64_t seed,
  const Transform & callback)
{
  std::uint64_t state = seed;
  for (std::uint64_t index = 0; index < iterations; ++index) {
    state = callback(state, index);
  }
  return state;
}

}  // namespace rclcppyy_boundary_benchmark

#endif  // RCLCPPYY_BENCHMARK_BOUNDARY_KERNEL_HPP

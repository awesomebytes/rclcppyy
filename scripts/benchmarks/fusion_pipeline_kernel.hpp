#ifndef RCLCPPYY_FUSION_PIPELINE_KERNEL_HPP
#define RCLCPPYY_FUSION_PIPELINE_KERNEL_HPP

#include <cstddef>
#include <cstdint>

namespace rclcppyy_fusion_pipeline {

constexpr std::size_t kStages = 4;

inline std::uint64_t stage(std::size_t index, std::uint64_t value)
{
  switch (index) {
    case 0: return value + 1ULL;
    case 1: return value * 3ULL;
    case 2: return value + 5ULL;
    case 3: return value ^ 0x5aULL;
    default: return value;
  }
}

inline std::uint64_t fused(std::uint64_t value)
{
  for (std::size_t index = 0; index < kStages; ++index) {
    value = stage(index, value);
  }
  return value;
}

}  // namespace rclcppyy_fusion_pipeline

#endif  // RCLCPPYY_FUSION_PIPELINE_KERNEL_HPP

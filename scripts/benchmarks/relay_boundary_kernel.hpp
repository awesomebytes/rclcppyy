#ifndef RCLCPPYY_RELAY_BOUNDARY_KERNEL_HPP
#define RCLCPPYY_RELAY_BOUNDARY_KERNEL_HPP

#include <cstdint>

namespace rclcppyy_relay_boundary {

inline std::uint64_t transform(std::uint64_t value)
{
  return value * 2ULL + 1ULL;
}

}  // namespace rclcppyy_relay_boundary

#endif  // RCLCPPYY_RELAY_BOUNDARY_KERNEL_HPP

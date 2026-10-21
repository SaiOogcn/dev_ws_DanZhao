#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace lio_aurora
{

constexpr double kPi = 3.14159265358979323846;

inline double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline Pose2d compose(const Pose2d & lhs, const Pose2d & rhs)
{
  const double c = std::cos(lhs.yaw);
  const double s = std::sin(lhs.yaw);
  return {
    lhs.x + c * rhs.x - s * rhs.y,
    lhs.y + s * rhs.x + c * rhs.y,
    normalize_angle(lhs.yaw + rhs.yaw)};
}

inline Pose2d inverse(const Pose2d & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {
    -c * pose.x - s * pose.y,
    s * pose.x - c * pose.y,
    normalize_angle(-pose.yaw)};
}

inline Eigen::Vector2d transform_point(const Pose2d & pose, const Eigen::Vector2d & point)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return Eigen::Vector2d(
    c * point.x() - s * point.y() + pose.x,
    s * point.x() + c * point.y() + pose.y);
}

}  // namespace lio_aurora

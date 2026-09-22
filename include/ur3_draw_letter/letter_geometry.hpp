#pragma once

#include <cmath>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

namespace ur3_draw_letter
{

inline std::vector<geometry_msgs::msg::Pose> make_letter_d_waypoints(
  double plane_y,
  double center_x,
  double bottom_z,
  double height,
  double width,
  int curve_waypoint_count,
  const geometry_msgs::msg::Quaternion & orientation)
{
  constexpr double kPi = 3.14159265358979323846;
  const double right_x = center_x + width / 2.0;
  const double middle_z = bottom_z + height / 2.0;

  std::vector<geometry_msgs::msg::Pose> points;
  points.reserve(static_cast<std::size_t>(curve_waypoint_count) + 2U);

  geometry_msgs::msg::Pose pose;
  pose.orientation = orientation;
  pose.position.x = right_x;
  pose.position.y = plane_y;
  pose.position.z = bottom_z;
  points.push_back(pose);

  pose.position.z = bottom_z + height;
  points.push_back(pose);

  for (int i = 1; i <= curve_waypoint_count; ++i) {
    const double angle = kPi / 2.0 -
      (kPi * static_cast<double>(i) / static_cast<double>(curve_waypoint_count));
    pose.position.x = right_x - width * std::cos(angle);
    pose.position.z = middle_z + (height / 2.0) * std::sin(angle);
    points.push_back(pose);
  }

  return points;
}

}  // namespace ur3_draw_letter

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "geometry_msgs/msg/quaternion.hpp"
#include "ur3_draw_letter/letter_geometry.hpp"

int main()
{
  constexpr double plane_y = 0.221;
  constexpr double center_x = 0.12;
  constexpr double bottom_z = 0.55;
  constexpr double height = 0.12;
  constexpr double width = 0.08;
  constexpr int curve_points = 35;

  geometry_msgs::msg::Quaternion orientation;
  orientation.w = 1.0;
  const auto points = ur3_draw_letter::make_letter_d_waypoints(
    plane_y, center_x, bottom_z, height, width, curve_points, orientation);

  assert(height > 0.0);
  assert(width > 0.0);
  assert(points.size() == static_cast<std::size_t>(curve_points + 2));

  double min_x = points.front().position.x;
  double max_x = min_x;
  double min_z = points.front().position.z;
  double max_z = min_z;
  for (const auto & point : points) {
    min_x = std::min(min_x, point.position.x);
    max_x = std::max(max_x, point.position.x);
    min_z = std::min(min_z, point.position.z);
    max_z = std::max(max_z, point.position.z);
    assert(std::abs(point.position.y - plane_y) < 1e-12);
  }

  // Cung dùng 35 mẫu nên không bắt buộc có mẫu đúng tại góc 0 rad;
  // sai số bounding-box do rời rạc hóa vẫn phải nhỏ hơn 0.1 mm.
  assert(std::abs((max_x - min_x) - width) < 1e-4);
  assert(std::abs((max_z - min_z) - height) < 1e-6);
  std::cout << "letter_geometry: PASS\n";
  return 0;
}

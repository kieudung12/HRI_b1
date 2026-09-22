#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "ur3_draw_letter/letter_geometry.hpp"

using moveit::planning_interface::MoveGroupInterface;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMarkerPointSpacing = 0.0005;
constexpr std::size_t kMarkerReserve = 250;
// 35 điểm đủ mô tả cung elip mượt hơn nhiều so với eef_step=5 mm,
// nhưng không tạo dư waypoint làm trajectory dài và nặng không cần thiết.
constexpr int kCurveWaypointCount = 35;
constexpr auto kPoseStateSettleTime = std::chrono::milliseconds(400);
constexpr auto kCartesianStateSettleTime = std::chrono::milliseconds(300);
constexpr auto kMarkerUpdatePeriod = std::chrono::milliseconds(40);

geometry_msgs::msg::Pose makePose(
  double x, double y, double z, const geometry_msgs::msg::Quaternion & orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;
  return pose;
}

void publishMarker(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & frame);

bool planAndExecutePose(
  MoveGroupInterface & move_group, const geometry_msgs::msg::Pose & target,
  const std::string & end_effector_link, const rclcpp::Logger & logger,
  const std::string & label)
{
  // Chờ controller ổn định rồi lấy trạng thái mới trước khi lập kế hoạch.
  std::this_thread::sleep_for(kPoseStateSettleTime);
  const auto current_state = move_group.getCurrentState(2.0);
  if (current_state) {
    move_group.setStartState(*current_state);
  } else {
    move_group.setStartStateToCurrentState();
  }
  // Giải IK ngay từ current_state rồi mới plan theo joint target. Nếu dùng
  // setPoseTarget(), OMPL có thể chọn một nhánh IK xa tư thế hiện tại; trong
  // Gazebo nhánh đó dễ tạo sai số lớn ở controller và làm robot tụt xuống.
  // Bộ giải IK đôi khi trả lời quá sớm khi MoveIt vẫn đang đồng bộ bản tin
  // joint_states đầu tiên, nhất là lúc RViz khởi động cùng thời điểm.
  // Thử lại cùng một đích xác định trong thời gian ngắn để không dừng cả bài
  // vẽ chỉ vì lỗi tạm thời của bộ giải.
  bool ik_ok = false;
  for (int attempt = 0; attempt < 10 && rclcpp::ok(); ++attempt) {
    ik_ok = move_group.setJointValueTarget(target, end_effector_link);
    if (ik_ok) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!ik_ok) {
    RCLCPP_ERROR(logger, "%s: khong tim duoc nghiem IK an toan.", label.c_str());
    return false;
  }

  // Một số lần IK trả về cùng tư thế nhưng với góc khớp lệch ±2π
  // (ví dụ -5.57 thay vì 0.71 rad). Controller sẽ đi vòng rất xa và dễ
  // vi phạm path tolerance. Đưa từng khớp về nghiệm gần trạng thái hiện tại.
  std::vector<double> joint_target;
  move_group.getJointValueTarget(joint_target);
  const auto current_joints = move_group.getCurrentJointValues();
  if (joint_target.size() == current_joints.size()) {
    for (std::size_t i = 0; i < joint_target.size(); ++i) {
      while (joint_target[i] - current_joints[i] > kPi) {
        joint_target[i] -= 2.0 * kPi;
      }
      while (joint_target[i] - current_joints[i] < -kPi) {
        joint_target[i] += 2.0 * kPi;
      }
    }
    if (!move_group.setJointValueTarget(joint_target)) {
      RCLCPP_ERROR(logger, "%s: khong dat duoc nghiem IK gan trang thai hien tai.", label.c_str());
      return false;
    }
  }

  MoveGroupInterface::Plan plan;
  const auto planning_result = move_group.plan(plan);
  move_group.clearPoseTargets();

  if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: planning that bai.", label.c_str());
    return false;
  }

  if (!plan.trajectory_.joint_trajectory.points.empty()) {
    const auto & final_point = plan.trajectory_.joint_trajectory.points.back();
    std::string target_values;
    for (std::size_t i = 0; i < final_point.positions.size(); ++i) {
      target_values += std::to_string(final_point.positions[i]);
      if (i + 1 < final_point.positions.size()) {
        target_values += ", ";
      }
    }
    RCLCPP_INFO(logger, "%s joint target: [%s]", label.c_str(), target_values.c_str());
  }

  RCLCPP_INFO(logger, "%s plan: SUCCESS; bat dau execute.", label.c_str());
  const auto execution_result = move_group.execute(plan);
  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: execution that bai.", label.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "%s execute: SUCCESS.", label.c_str());
  return true;
}

double executeCartesian(
  MoveGroupInterface & move_group, const std::vector<geometry_msgs::msg::Pose> & waypoints,
  double eef_step, double jump_threshold, double minimum_fraction,
  const rclcpp::Logger & logger,
  const std::string & label,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & marker_publisher,
  tf2_ros::Buffer * tf_buffer, const std::string & marker_frame,
  const std::string & end_effector_link, bool track_tcp)
{
  // Cho Gazebo co thoi gian cap nhat joint_states sau khi action truoc ket thuc.
  // Lay mot state moi roi dung chinh state do lam diem bat dau Cartesian.
  std::this_thread::sleep_for(kCartesianStateSettleTime);
  const auto current_state = move_group.getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(logger, "%s: khong lay duoc robot state moi.", label.c_str());
    return 0.0;
  }
  move_group.setStartState(*current_state);

  moveit_msgs::msg::RobotTrajectory trajectory;
  // HOVER không đi qua hàm này; nó dùng normal pose planning ở bên dưới.
  // Ba đoạn còn lại đều là Cartesian path và giữ collision checking bật.
  const double fraction = move_group.computeCartesianPath(
    waypoints, eef_step, 0.0, trajectory, true);

  double max_joint_jump = 0.0;
  std::size_t max_jump_index = 0;
  std::size_t max_jump_joint = 0;
  if (trajectory.joint_trajectory.points.size() > 1) {
    for (std::size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
      const auto & previous = trajectory.joint_trajectory.points[i - 1].positions;
      const auto & current = trajectory.joint_trajectory.points[i].positions;
      const std::size_t joint_count = std::min(previous.size(), current.size());
      for (std::size_t j = 0; j < joint_count; ++j) {
        const double jump = std::abs(current[j] - previous[j]);
        if (jump > max_joint_jump) {
          max_joint_jump = jump;
          max_jump_index = i;
          max_jump_joint = j;
        }
      }
    }
    const std::string joint_name =
      max_jump_joint < trajectory.joint_trajectory.joint_names.size() ?
      trajectory.joint_trajectory.joint_names[max_jump_joint] : "unknown";
    RCLCPP_INFO(
      logger, "%s trajectory: %zu points, max joint step %.3f rad (%s, point %zu).",
      label.c_str(), trajectory.joint_trajectory.points.size(), max_joint_jump,
      joint_name.c_str(), max_jump_index);
  }

  if (label == "LETTER D" && max_joint_jump > jump_threshold) {
    RCLCPP_ERROR(
      logger,
      "%s bi tu choi: buoc khop lon nhat %.3f rad vuot gioi han %.3f rad.",
      label.c_str(), max_joint_jump, jump_threshold);
    return 0.0;
  }

  RCLCPP_INFO(
    logger, "%s Cartesian fraction: %.1f%%", label.c_str(), fraction * 100.0);

  if (fraction < minimum_fraction) {
    RCLCPP_ERROR(
      logger, "%s bi tu choi: fraction %.3f < nguong %.3f.",
      label.c_str(), fraction, minimum_fraction);
    return fraction;
  }

  // Hiển thị điểm bắt đầu ngay trước khi gửi trajectory. Nếu chờ thread TF
  // đầu tiên, RViz có thể chỉ thấy TCP đã đi một đoạn rồi mới thấy nét.
  if (track_tcp && marker_publisher && !waypoints.empty()) {
    geometry_msgs::msg::Point first_point;
    first_point.x = waypoints.front().position.x;
    first_point.y = waypoints.front().position.y;
    first_point.z = waypoints.front().position.z;
    publishMarker(
      marker_publisher, std::vector<geometry_msgs::msg::Point>{first_point}, marker_frame);
  }

  std::atomic_bool tracking{true};
  std::thread tracker;
  if (track_tcp && marker_publisher && tf_buffer) {
    tracker = std::thread([
        &tracking, marker_publisher, tf_buffer, marker_frame, end_effector_link, logger]() {
      std::vector<geometry_msgs::msg::Point> actual_points;
      actual_points.reserve(kMarkerReserve);
      geometry_msgs::msg::Point last_point;
      bool have_last_point = false;

      while (tracking.load() && rclcpp::ok()) {
        try {
          const auto transform = tf_buffer->lookupTransform(
            marker_frame, end_effector_link, tf2::TimePointZero);
          geometry_msgs::msg::Point point;
          point.x = transform.transform.translation.x;
          point.y = transform.transform.translation.y;
          point.z = transform.transform.translation.z;

          const double dx = point.x - last_point.x;
          const double dy = point.y - last_point.y;
          const double dz = point.z - last_point.z;
          const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
          if (!have_last_point || distance >= kMarkerPointSpacing) {
            actual_points.push_back(point);
            last_point = point;
            have_last_point = true;
            publishMarker(marker_publisher, actual_points, marker_frame);
          }
        } catch (const tf2::TransformException &) {
          // TF có thể chưa cập nhật trong một vài chu kỳ đầu; tiếp tục thử.
        }
        std::this_thread::sleep_for(kMarkerUpdatePeriod);
      }

      // Lấy thêm vị trí cuối để nét vẽ kết thúc đúng tại TCP.
      try {
        const auto transform = tf_buffer->lookupTransform(
          marker_frame, end_effector_link, tf2::TimePointZero);
        geometry_msgs::msg::Point point;
        point.x = transform.transform.translation.x;
        point.y = transform.transform.translation.y;
        point.z = transform.transform.translation.z;
        if (!have_last_point ||
          std::hypot(point.x - last_point.x, point.y - last_point.y) >=
          kMarkerPointSpacing ||
          std::abs(point.z - last_point.z) >= kMarkerPointSpacing)
        {
          actual_points.push_back(point);
          publishMarker(marker_publisher, actual_points, marker_frame);
        }
      } catch (const tf2::TransformException &) {
        RCLCPP_WARN(logger, "Khong lay duoc TF cuoi cung cua TCP de cap nhat marker.");
      }
      RCLCPP_INFO(logger, "Marker TCP da cap nhat %zu diem theo vi tri thuc te.", actual_points.size());
    });
  }

  const auto result = move_group.execute(trajectory);
  tracking.store(false);
  if (tracker.joinable()) {
    tracker.join();
  }
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: execution that bai.", label.c_str());
    return 0.0;
  }

  RCLCPP_INFO(logger, "%s: execution thanh cong.", label.c_str());
  return fraction;
}

void publishMarker(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & frame)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  // Dùng TF mới nhất; các điểm đã nằm trong hệ tọa độ world.
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = "letter_d";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.006;
  marker.color.r = 0.1F;
  marker.color.g = 0.9F;
  marker.color.b = 0.2F;
  marker.color.a = 1.0F;

  marker.points = points;

  publisher->publish(marker);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  // Mo phong Gazebo dung /clock; mac dinh bat sim time de node chay duoc ca
  // khi duoc goi truc tiep o terminal 2, khong chi qua launch file.
  node_options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  const auto node = rclcpp::Node::make_shared("draw_letter_d_node", node_options);
  const auto logger = node->get_logger();

  // Mặt phẳng XZ cùng hướng tự nhiên của TCP ở tư thế khởi động. Đặt nó
  // gần y=0.221 để chữ nhích ra ngoài robot nhưng vẫn giữ nhánh IK ổn định.
  const double plane_y = node->declare_parameter("plane_y", 0.221);
  const double center_x = node->declare_parameter("center_x", 0.12);
  const double bottom_z = node->declare_parameter("bottom_z", 0.55);
  const double letter_height = node->declare_parameter("letter_height", 0.12);
  const double letter_width = node->declare_parameter("letter_width", 0.08);
  const double lift_distance = node->declare_parameter("lift_distance", 0.05);
  // Mặc định phải giống launch file để chạy node trực tiếp cũng giữ được
  // tốc độ ổn định khi Gazebo GUI đang mở.
  const double velocity_scaling = node->declare_parameter("velocity_scaling", 0.05);
  const double acceleration_scaling = node->declare_parameter("acceleration_scaling", 0.05);
  const double minimum_fraction = 1.0;  // Chỉ thực thi khi lập được toàn bộ đường đi.
  // Mẫu 5 mm giúp bộ giải IK bám cùng nhánh qua đoạn đứng đầu chữ D;
  // 10 mm đôi khi nhảy nghiệm ngay sau vài mẫu đầu.
  const double eef_step = node->declare_parameter("eef_step", 0.005);
  // Nét D dùng sampling 5 mm và có kiểm tra bước khớp tuyệt đối trong
  // executeCartesian() dùng jump threshold = 0.0 của MoveIt và kiểm tra
  // bước khớp tuyệt đối riêng cho LETTER D; collision checking vẫn bật.
  const double jump_threshold = node->declare_parameter("jump_threshold", 0.5);

  auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  const auto marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>(
    "/drawing_path", marker_qos);
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  RCLCPP_INFO(logger, "Khoi tao MoveGroupInterface cho group ur_manipulator.");
  MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(std::clamp(velocity_scaling, 0.01, 1.0));
  move_group.setMaxAccelerationScalingFactor(std::clamp(acceleration_scaling, 0.01, 1.0));

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End-effector link: %s", move_group.getEndEffectorLink().c_str());
  const std::string end_effector_link = move_group.getEndEffectorLink();

  bool overall_success = true;
  for (int attempt = 0; attempt < 20 && rclcpp::ok(); ++attempt) {
    if (move_group.getCurrentState(1.0)) {
      break;
    }
    if (attempt == 19) {
      RCLCPP_ERROR(logger, "Khong nhan duoc robot state tu MoveIt.");
      overall_success = false;
    }
  }

  geometry_msgs::msg::Quaternion orientation;
  // TCP vẫn vuông góc mặt phẳng XZ (pháp tuyến gần +Y), nhưng xoay nhẹ
  // quanh pháp tuyến để tránh tư thế kỳ dị khi đi dọc chân chữ D.
  constexpr double tool_roll = 0.20;  // rad, khoảng 11 độ
  const double half_roll = tool_roll / 2.0;
  orientation.x = -std::cos(half_roll) * std::sqrt(0.5);
  orientation.y = std::sin(half_roll) * std::sqrt(0.5);
  orientation.z = -std::sin(half_roll) * std::sqrt(0.5);
  orientation.w = std::cos(half_roll) * std::sqrt(0.5);

  const double right_x = center_x + letter_width / 2.0;
  const double start_z = bottom_z;
  const auto hover_pose = makePose(
    right_x, plane_y + lift_distance, start_z, orientation);
  const auto pen_down_pose = makePose(right_x, plane_y, start_z, orientation);
  const auto letter_points = ur3_draw_letter::make_letter_d_waypoints(
    plane_y, center_x, bottom_z, letter_height, letter_width,
    kCurveWaypointCount, orientation);
  const auto retract_pose = makePose(
    right_x, plane_y + lift_distance, start_z, orientation);

  // Thực hiện đúng một lần: tiếp cận, hạ bút, viết D, nhấc bút.
  const auto cartesian = [&](const std::vector<geometry_msgs::msg::Pose> & poses,
      const std::string & label, bool draw = false) {
    return executeCartesian(
      move_group, poses, eef_step, jump_threshold, minimum_fraction, logger, label,
      marker_publisher, &tf_buffer, move_group.getPlanningFrame(), end_effector_link,
      draw) >= minimum_fraction;
  };

  if (overall_success && rclcpp::ok()) {
    RCLCPP_INFO(logger, "READY: controller recovery hold da thanh cong.");
    // Tiếp cận điểm hover bằng joint-space sau khi đã giải IK từ trạng thái
    // hiện tại; phần viết chữ phía sau vẫn là Cartesian path liên tục.
    overall_success = planAndExecutePose(
      move_group, hover_pose, end_effector_link, logger, "HOVER");
  }
  if (overall_success && rclcpp::ok()) {
    overall_success = cartesian({pen_down_pose}, "PEN DOWN");
  }
  if (overall_success && rclcpp::ok()) {
    overall_success = cartesian(letter_points, "LETTER D", true);
  }
  // Chỉ nhấc bút theo đường đã thiết kế khi chữ D thực thi thành công.
  // Nếu lỗi giữa nét, dừng để tránh đi chéo qua mặt phẳng vẽ.
  if (overall_success && rclcpp::ok()) {
    overall_success = cartesian({retract_pose}, "PEN UP");
  }

  if (overall_success && rclcpp::ok()) {
    RCLCPP_INFO(logger, "Hoan tat chu D mot lan. Giu net ve tren RViz; Ctrl+C de thoat.");
    // Chỉ giữ publisher sống để RViz mở muộn vẫn nhận được nét cuối.
    // Không gửi thêm bất kỳ chuyển động nào.
    spinner.join();
  } else {
    RCLCPP_ERROR(logger, "Dung bai ve vi chuyen dong khong hoan tat.");
    executor.cancel();
    spinner.join();
  }
  rclcpp::shutdown();
  return overall_success ? 0 : 1;
}

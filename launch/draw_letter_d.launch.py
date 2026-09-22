from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("draw_gazebo_gui")
    # Tên riêng để không bị hai launch file UR bên trong ghi đè. Các launch
    # chính thức cũng có argument `launch_rviz`, nhưng chúng ta luôn tắt RViz
    # của chúng và mở file RViz riêng của bài.
    draw_rviz = LaunchConfiguration("draw_rviz")
    draw_node_enabled = LaunchConfiguration("draw_node")
    ign_partition = LaunchConfiguration("ign_partition")

    # Dùng launch Gazebo chính thức với GUI để world được tạo trước khi node
    # spawn UR3; đây là thứ tự ổn định nhất cho gz_ros2_control.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "runtime_config_package": "ur3_draw_letter",
            "controllers_file": "ur_controllers_assignment.yaml",
            # Chỉ có một server vật lý và bridge /clock.
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
        }.items(),
    )

    # MoveIt được khởi động độc lập để có thể tắt RViz mà vẫn giữ
    # move_group/planning/execution hoạt động bình thường.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_servo": "false",
            # RViz la giao dien MoveIt cua bai; truyen truc tiep de tranh
            # LaunchConfiguration bi mat khi long launch file.
            "launch_rviz": "false",
        }.items(),
    )

    drawing_node = Node(
        package="ur3_draw_letter",
        executable="draw_letter_d_node",
        name="draw_letter_d_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            # Gazebo GUI làm simulation nặng hơn headless; tốc độ vừa phải
            # giúp joint trajectory controller bám sát quỹ đạo, tránh rơi
            # khỏi path tolerance khi đưa TCP tới mặt phẳng viết.
            {"velocity_scaling": 0.05, "acceleration_scaling": 0.05},
            # Dùng cùng bộ giải IK chính thức với move_group để node vẽ
            # không phải rơi về cấu hình kinematics mặc định.
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
            ),
        ],
        condition=IfCondition(draw_node_enabled),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_draw_letter",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("ur3_draw_letter"), "config", "draw_letter.rviz"]
            ),
        ],
        parameters=[
            {"use_sim_time": True},
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
            ),
        ],
        condition=IfCondition(draw_rviz),
    )

    # Một số máy khởi động gz_ros2_control chậm hơn controller_manager.
    # Khi đó spawner của launch chính thức có thể chạy quá sớm và robot rơi.
    # Chờ Gazebo ổn định rồi kiểm tra/nạp lại hai controller bắt buộc.
    controller_recovery_process = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            """set -u
for attempt in $(seq 1 60); do
  if timeout 1 ros2 service type /controller_manager/list_controllers >/dev/null 2>&1; then
    status=$(timeout 2 ros2 control list_controllers 2>/dev/null | sed $'s/\\033\\[[0-9;]*m//g')
    if grep -q '^joint_state_broadcaster[[:space:]].*active' <<<"${status}" && \
       grep -q '^joint_trajectory_controller[[:space:]].*active' <<<"${status}"; then
      # Gazebo da dat joint ve initial_value, nhung JTC can nhan mot goal
      # dau tien de giu cac khop o tu the do; neu khong, trong luc co the
      # keo canh tay xuong truoc khi MoveIt gui trajectory dau tien.
      echo "[controller-recovery] Giữ robot ở tư thế khởi động an toàn."
      if ros2 action send_goal -f \
        /joint_trajectory_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory \
        "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], time_from_start: {sec: 1, nanosec: 0}}]}}" \
        >/dev/null 2>&1; then
        echo "[READY] controller hold execute: SUCCESS; bat dau node ve."
        exit 0
      fi
      echo "[controller-recovery] Không giữ được tư thế khởi động." >&2
      exit 1
    fi
    for controller in joint_state_broadcaster joint_trajectory_controller; do
      if ! grep -q "^${controller}[[:space:]]" <<<"${status}"; then
        ros2 run controller_manager spawner "${controller}" -c /controller_manager \
          --controller-manager-timeout 3 >/dev/null 2>&1 || true
      elif ! grep -q "^${controller}[[:space:]].*active" <<<"${status}"; then
        ros2 control set_controller_state "${controller}" active \
          -c /controller_manager >/dev/null 2>&1 || true
      fi
    done
  fi
  sleep 0.25
done
echo "[controller-recovery] Khong kich hoat duoc controller sau 15 giay." >&2
exit 1
""",
        ],
        output="screen",
    )
    controller_recovery = TimerAction(
        period=2.0,
        actions=[controller_recovery_process],
    )

    # `ur_sim_control.launch.py` starts Gazebo and ros_gz_sim/create at the
    # same time. On a loaded machine create can return before the world is
    # ready, leaving Gazebo without the robot. Retry only when /joint_states
    # is still absent, so an already spawned robot is never duplicated.
    spawn_retry_process = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            """set -u
for attempt in $(seq 1 40); do
  if timeout 2 ros2 topic list 2>/dev/null | grep -qx '/joint_states'; then
    exit 0
  fi
  if timeout 2 ros2 service list 2>/dev/null | grep -qx '/world/empty/create'; then
    if timeout 10 ros2 run ros_gz_sim create -topic robot_description -name ur -allow_renaming true >/dev/null 2>&1; then
      exit 0
    fi
  fi
  sleep 0.5
done
exit 0
""",
        ],
        output="screen",
    )
    spawn_retry = TimerAction(
        period=6.0,
        actions=[spawn_retry_process],
    )

    def start_drawing_after_success(event, _context):
        # Khong cho node ve chay neu recovery that bai hoac bi dung bang Ctrl+C.
        if event.returncode == 0:
            return [drawing_node]
        return []

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument(
                "draw_gazebo_gui",
                default_value="true",
                description="Mo Gazebo GUI client sau khi server da san sang.",
            ),
            DeclareLaunchArgument(
                "draw_rviz",
                default_value="true",
                description="Mo RViz MoveIt.",
            ),
            DeclareLaunchArgument(
                "draw_node",
                default_value="true",
                description="Chay node ve chu D sau khi controller san sang.",
            ),
            DeclareLaunchArgument(
                "ign_partition",
                default_value="ur3_draw_letter",
                description="Partition rieng cho Gazebo de tranh trung /clock.",
            ),
            SetEnvironmentVariable("IGN_PARTITION", ign_partition),
            simulation,
            moveit,
            rviz_node,
            spawn_retry,
            controller_recovery,
            # Chi chay node ve sau khi controller recovery ket thuc thanh cong.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controller_recovery_process,
                    on_exit=start_drawing_after_success,
                )
            ),
        ]
    )

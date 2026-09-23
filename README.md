# UR3e Cartesian Letter Drawing using ROS 2 and MoveIt 2

**Human–Robot Interaction — Assignment 01**
Điều khiển robot UR3e viết chữ **D** trong Gazebo bằng ROS 2 Humble và MoveIt 2.

Chữ D là chữ cái đầu trong tên **Dũng**. Package tập trung vào việc xây dựng
Cartesian waypoint, lập kế hoạch chuyển động bằng MoveIt 2 và thực thi
trajectory qua `joint_trajectory_controller`.

## 1. Overview

Mục tiêu của bài là làm quen với một pipeline điều khiển robot công nghiệp:

- mô phỏng UR3e trong Gazebo Fortress;
- nhận trạng thái khớp và TF từ simulation;
- dùng MoveIt 2 cho inverse kinematics, planning và collision checking;
- chuyển các waypoint Cartesian của chữ D thành joint trajectory;
- quan sát robot và đường đi TCP trong RViz.

Robot thực hiện một lần theo trình tự:

```text
READY → HOVER → PEN DOWN → LETTER D → PEN UP
```

## 2. System Architecture

```mermaid
flowchart TD
    A[Launch file] --> B[UR3e Gazebo Simulation]
    B --> C[ros2_control Controllers]
    C --> D[Joint states and TF]
    D --> E[MoveIt 2 / move_group]
    F[draw_letter_d_node] --> G[Cartesian Waypoints]
    G --> E
    E --> H[JointTrajectory]
    H --> I[joint_trajectory_controller]
    I --> B

    D --> F
    F --> J[/drawing_path Marker]
    J --> K[RViz]
```

`draw_letter_d.launch.py` khởi động simulation, controller, MoveIt, RViz và
node vẽ theo đúng thứ tự phụ thuộc. Controller recovery chờ
`joint_state_broadcaster` và `joint_trajectory_controller` ở trạng thái
`active` trước khi cho node bắt đầu lập kế hoạch.

## 3. Motion Sequence

| State | Description |
|---|---|
| `READY` | Recovery gửi goal giữ robot ở tư thế khởi động an toàn. |
| `HOVER` | MoveIt giải IK và lập kế hoạch joint-space đến phía trên điểm bắt đầu. |
| `PEN DOWN` | TCP hạ xuống mặt phẳng viết bằng Cartesian path. |
| `LETTER D` | TCP đi theo toàn bộ waypoint của chữ D. |
| `PEN UP` | TCP nhấc lên khỏi mặt phẳng sau khi viết xong. |

`HOVER` dùng pose planning vì đây là chuyển động tiếp cận. Ba đoạn còn lại
dùng Cartesian path để TCP bám trực tiếp theo mặt phẳng và hình dạng chữ.

## 4. Cartesian Letter-D Generation

Chữ D được vẽ trong frame `world`, trên mặt phẳng XZ với tọa độ Y cố định:

```text
                 z
                 ↑
        đỉnh  ┌──┐
              │  )
              │  )  ← nửa ellipse
        đáy   └──┘
                 └────────→ x

                 y = 0.221 m
```

Các thông số hình học hiện tại:

- `center_x = 0.12 m`;
- `bottom_z = 0.55 m`;
- `height = 0.12 m`;
- `width = 0.08 m`;
- `plane_y = 0.221 m`.

TCP giữ orientation cố định trong khi viết. Chữ gồm một đoạn thẳng từ chân
phải lên đỉnh phải, sau đó là nửa ellipse quay về chân phải.

Vị trí chân phải được tính bởi:

```text
x_right  = center_x + width / 2
z_bottom = bottom_z
```

Các điểm trên cung cong được nội suy theo:

```text
x(theta) = x_right - width * cos(theta)
z(theta) = bottom_z + height / 2
           + height / 2 * sin(theta)
```

Waypoint chỉ thay đổi X và Z; Y giữ nguyên bằng `plane_y`. Vì vậy đường đi
Cartesian nằm trên cùng một mặt phẳng trong suốt quá trình viết.

## 5. MoveIt 2 Planning and Safety

MoveIt 2 nhận pose/waypoint, giải inverse kinematics và tạo
`JointTrajectory` cho controller. Package áp dụng các điều kiện an toàn sau:

- `HOVER` dùng pose planning; `PEN DOWN`, `LETTER D` và `PEN UP` dùng
  `computeCartesianPath()`;
- collision checking được bật khi tính Cartesian path;
- Cartesian fraction phải đạt `1.0` trước khi trajectory được execute;
- MoveIt relative jump threshold được truyền bằng `0.0`;
- code có custom limit `max_absolute_joint_step = 0.5 rad` để kiểm tra bước
  nhảy tuyệt đối giữa các mẫu khớp;
- `velocity_scaling = 0.05` và `acceleration_scaling = 0.05`;
- controller dùng trajectory tolerance `0.30 rad` để ổn định trong Gazebo;
- nghiệm IK của HOVER được đưa về gần trạng thái khớp hiện tại để tránh đi
  vòng thêm `2π` ở một khớp.

Các tham số hình học và planning chính:

| Parameter | Default | Meaning |
|---|---:|---|
| `plane_y` | `0.221 m` | Vị trí mặt phẳng viết theo trục Y. |
| `center_x` | `0.12 m` | Tọa độ X tại tâm chữ. |
| `bottom_z` | `0.55 m` | Độ cao đáy chữ. |
| `letter_height` | `0.12 m` | Chiều cao chữ D. |
| `letter_width` | `0.08 m` | Chiều rộng chữ D. |
| `lift_distance` | `0.05 m` | Khoảng TCP được nâng trước/sau khi viết. |
| `eef_step` | `0.005 m` | Bước lấy mẫu Cartesian, tương đương 5 mm. |
| `max_absolute_joint_step` | `0.5 rad` | Giới hạn bước nhảy khớp tuyệt đối. |
| velocity / acceleration scaling | `0.05` | Giới hạn vận tốc và gia tốc của MoveIt. |

## 6. Package Structure

```text
ur3_draw_letter/
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── README.md
├── config/
│   ├── draw_letter.rviz
│   └── ur_controllers_assignment.yaml
├── docs/images/
│   └── .gitkeep
├── include/ur3_draw_letter/
│   └── letter_geometry.hpp
├── launch/
│   └── draw_letter_d.launch.py
├── scripts/
│   └── validate_runtime.sh
├── src/
│   └── draw_letter_d_node.cpp
└── test/
    └── test_letter_geometry.cpp
```

- `launch/draw_letter_d.launch.py`: khởi động toàn bộ hệ thống.
- `src/draw_letter_d_node.cpp`: MoveIt planning, execution và marker TCP.
- `include/.../letter_geometry.hpp`: sinh waypoint hình học của chữ D.
- `config/ur_controllers_assignment.yaml`: cấu hình controller mô phỏng.
- `config/draw_letter.rviz`: Fixed Frame, RobotModel, MoveIt và Marker.
- `scripts/validate_runtime.sh`: kiểm tra runtime sau khi launch.
- `test/test_letter_geometry.cpp`: kiểm tra mặt phẳng và kích thước waypoint.

## 7. Requirements

- Ubuntu 22.04 hoặc WSL2 Ubuntu 22.04 có hỗ trợ GUI;
- ROS 2 Humble Desktop;
- MoveIt 2;
- Gazebo Fortress;
- `ur_description`;
- `ur_moveit_config`;
- `ur_simulation_gz`.

Simulation không cần robot UR thật và không cần chạy Universal Robots ROS
Driver. Các package UR cần được cài theo tài liệu chính thức trước khi build.

## 8. Build

Ví dụ clone package vào workspace ROS 2:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kieudung12/HRI_b1.git ur3_draw_letter

cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src \
  --ignore-src -r -y --rosdistro humble

colcon build \
  --packages-select ur3_draw_letter \
  --symlink-install

source install/setup.bash
```

## 9. Run

Chạy simulation, Gazebo, RViz và node viết chữ:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_draw_letter draw_letter_d.launch.py
```

Chạy headless để kiểm tra terminal:

```bash
ros2 launch ur3_draw_letter draw_letter_d.launch.py \
  draw_gazebo_gui:=false draw_rviz:=false
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur3e` | Loại robot Universal Robots. |
| `draw_gazebo_gui` | `true` | Bật/tắt cửa sổ Gazebo. |
| `draw_rviz` | `true` | Bật/tắt RViz. |
| `draw_node` | `true` | Bật/tắt node viết chữ. |
| `ign_partition` | `ur3_draw_letter` | Partition Gazebo riêng cho package. |

Chỉ nên chạy một launch Gazebo tại một thời điểm để tránh trùng `/clock` và
`/controller_manager`.

## 10. Validation

Sau khi launch đang chạy, mở terminal khác:

```bash
cd ~/ros2_ws/src/ur3_draw_letter
./scripts/validate_runtime.sh
```

Kiểm tra thủ công:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic info /drawing_path
ros2 topic info /clock
```

Expected controller state:

```text
joint_trajectory_controller   active
joint_state_broadcaster       active
```

Expected log chính:

```text
READY: controller recovery hold da thanh cong.
HOVER plan: SUCCESS
PEN DOWN Cartesian fraction: 100.0%
LETTER D Cartesian fraction: 100.0%
LETTER D: execution thanh cong.
PEN UP: execution thanh cong.
Hoan tat chu D mot lan.
```

## 11. Results

Ảnh minh họa sẽ được bổ sung sau khi chụp từ lần chạy cuối. Không đưa ảnh giả
vào repository:

<!-- Add screenshot here after creating docs/images/gazebo_result.png. -->
<!-- ![UR3e simulation](docs/images/gazebo_result.png) -->

<!-- Add screenshot here after creating docs/images/rviz_result.png. -->
<!-- ![Letter D trajectory in RViz](docs/images/rviz_result.png) -->

## 12. Demo

- Video demo: **Add a public Google Drive link here**.
- GitHub repository: <https://github.com/kieudung12/HRI_b1>

Không thêm link video cho đến khi video được upload và cấp quyền public.

## 13. References

- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Universal Robots ROS 2 Gazebo Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation)
- [Universal Robots ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [MoveIt examples for Universal Robots](https://github.com/dominikbelter/ros2_ur_moveit_examples)

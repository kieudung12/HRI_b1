# ur3_draw_letter

Package ROS 2 cho Bài thực hành 01: dùng UR3e, Gazebo và MoveIt 2 để viết
chữ cái đầu tiên trong tên **Dũng**, tức chữ **D**.

## 1. Mục tiêu và cách hoạt động

Launch file khởi động:

1. UR3e simulation trong Gazebo Fortress.
2. `joint_state_broadcaster` và `joint_trajectory_controller`.
3. MoveIt 2 và RViz.
4. Node điều khiển chữ D của sinh viên.

Node thực thi đúng một lần theo trình tự:

```text
giữ tư thế khởi động
        ↓
HOVER → PEN DOWN → LETTER D → PEN UP
```

- `READY`: controller recovery giữ robot ở tư thế khởi động.
- `HOVER`: MoveIt lập kế hoạch pose thông thường để đưa TCP tới phía trên
  điểm bắt đầu.
- `PEN DOWN`, `LETTER D`, `PEN UP`: MoveIt lập kế hoạch Cartesian cho ba
  đoạn waypoint.

Chuyển động được lập kế hoạch bằng MoveIt 2. Nét chữ được tạo từ Cartesian
waypoint và chỉ được thực thi khi Cartesian fraction đạt 100%.

## 2. Đối chiếu yêu cầu đề bài

- Robot: UR3e simulation.
- Mặt phẳng viết: mặt phẳng XZ trong frame `world`, tại `y = 0.223 m`.
  Đây là hướng tự nhiên của TCP ở tư thế home nên đường tiếp cận ngắn và ít
  đổi nhánh IK hơn.
- Tâm chữ: `x = 0.12 m`; đáy chữ ở `z = 0.55 m`, cao `0.12 m`, nên toàn bộ
  nét nằm cao hơn mặt đất và tránh vùng thân robot.
- Kích thước chữ: cao `0.12 m`, rộng `0.08 m`; tỉ lệ này đủ rõ trong RViz/Gazebo
  và giữ cung cong cách xa vùng self-collision của robot.
- Waypoint: một nét liên tục tối ưu gồm đoạn thẳng từ chân phải lên đỉnh phải,
  sau đó là nửa elip bên trái quay về chân phải. Không có đoạn di chuyển chéo
  qua mặt phẳng khi đang chạm bút.
- Điều khiển: MoveIt 2 lập kế hoạch và gửi joint trajectory đến controller.
- An toàn: MoveIt kiểm tra collision khi tính Cartesian path; quỹ đạo chỉ chạy
  khi fraction đạt `1.0`. Nét D dùng `eef_step = 0.005 m` và kiểm tra bước
  khớp tuyệt đối không quá `0.5 rad`; nghiệm IK của HOVER được chuẩn hóa về
  nhánh gần tư thế hiện tại để tránh quỹ đạo đi vòng `2π`. Nét D dùng
  `jump_threshold = 0.0` trong MoveIt để tránh heuristic tương đối loại nhầm
  mẫu đầu; code vẫn chặn bước khớp tuyệt đối quá `0.5 rad` trước khi execute.
- Controller dùng `trajectory tolerance = 0.30 rad`, đã kiểm tra ổn định trong
  Gazebo.
- Hiển thị: RViz tự nạp Marker ở topic `/drawing_path`.
- Marker lấy vị trí TCP thực tế từ TF trong lúc robot đang viết, nên nét xanh
  xuất hiện dần theo chuyển động của robot và không cần thêm Marker thủ công.

## 3. Cấu trúc package

```text
ur3/
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── README.md
├── config/
│   ├── draw_letter.rviz
│   └── ur_controllers_assignment.yaml
├── launch/
│   └── draw_letter_d.launch.py
├── include/ur3_draw_letter/
│   └── letter_geometry.hpp
├── scripts/
│   └── validate_runtime.sh
├── test/
│   └── test_letter_geometry.cpp
└── src/
    └── draw_letter_d_node.cpp
```

Các thư mục `build/`, `install/`, `log/` và `__pycache__/` là file sinh tự
động, đã được loại khỏi Git bằng `.gitignore`.

## 4. Phụ thuộc

Môi trường kiểm thử:

- Ubuntu 22.04 hoặc WSL2 Ubuntu 22.04 có hỗ trợ GUI.
- ROS 2 Humble Desktop.
- MoveIt 2.
- Gazebo Fortress / `ur_simulation_gz`.
- Các package UR Humble: `ur_description`, `ur_moveit_config` và
  `ur_simulation_gz`.

Cài các package UR theo tài liệu chính thức của Universal Robots. Với
simulation, Universal Robots ROS Driver không cần chạy robot thật.

## 5. Build

Giả sử package được đặt tại `~/ros2_ws/src/ur3`:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src/ur3 \
  --ignore-src -r -y --rosdistro humble

colcon build \
  --base-paths src/ur3 \
  --packages-select ur3_draw_letter \
  --symlink-install

source install/setup.bash
```

## 6. Chạy simulation

Chỉ chạy một launch của package trong một thời điểm:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_draw_letter draw_letter_d.launch.py
```

Lệnh trên mở Gazebo và RViz. Launch tự chờ hai controller ở trạng thái
`active`, gửi một goal giữ tư thế khởi động rồi mới khởi động node vẽ. Không
có delay cố định 10 hoặc 20 giây.

Có thể chạy không giao diện để kiểm tra terminal:

```bash
ros2 launch ur3_draw_letter draw_letter_d.launch.py \
  draw_gazebo_gui:=false draw_rviz:=false
```

Các launch argument:

- `ur_type`: mặc định `ur3e`.
- `draw_gazebo_gui`: mặc định `true`.
- `draw_rviz`: mặc định `true`.
- `draw_node`: mặc định `true`; đặt `false` nếu chỉ muốn kiểm tra simulation,
  controller và TF.
- `ign_partition`: mặc định `ur3_draw_letter`, giúp tách `/clock` khỏi các
  phiên Gazebo khác.

## 7. Runtime validation

Sau khi launch đang chạy, mở terminal khác:

```bash
cd ~/ros2_ws/src/ur3
./scripts/validate_runtime.sh
```

Script in trạng thái ROS distro, controller, MoveIt, joint state, action và
`/drawing_path`, sau đó kết luận `[PASS]` hoặc `[FAIL]`.

## 8. Kiểm tra kết quả

Trong terminal khác:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic info /drawing_path
ros2 topic info /clock
```

Kết quả đúng cần có:

```text
joint_trajectory_controller   active
joint_state_broadcaster       active
```

Log đúng của node sẽ lần lượt có các dòng tương tự:

```text
READY: controller recovery hold da thanh cong.
HOVER plan: SUCCESS
HOVER execute: SUCCESS.
PEN DOWN Cartesian fraction: 100.0%
LETTER D Cartesian fraction: 100.0%
LETTER D: execution thanh cong.
PEN UP: execution thanh cong.
Hoan tat chu D mot lan.
```

`ros2 topic info /clock` chỉ nên có một publisher. Launch đã đặt
`IGN_PARTITION` riêng; nếu vẫn có hai publisher hoặc log báo
`Detected jump back in time`, hãy đóng các launch/Gazebo cũ bằng `Ctrl+C` rồi
chạy lại một launch duy nhất.

## 9. Chuẩn bị nộp GitHub

Trước khi commit, kiểm tra:

```bash
cd ~/ros2_ws/src/ur3
find . -maxdepth 2 -type f | sort
git status
```

Chỉ cần nộp mã nguồn, cấu hình, launch, README và LICENSE. Không commit
`build/`, `install/`, `log/` hoặc `__pycache__/`.

Nếu tài khoản GitHub của bạn dùng username khác `dung`, hãy thay email
maintainer trong `package.xml` bằng địa chỉ GitHub noreply tương ứng.

## 10. Tài liệu tham khảo

- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Universal Robots ROS 2 Gazebo Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation)
- [Universal Robots ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [MoveIt examples for Universal Robots](https://github.com/dominikbelter/ros2_ur_moveit_examples)

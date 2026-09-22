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
READY → HOVER → PEN DOWN → LETTER D → PEN UP
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
- Mặt phẳng viết: mặt phẳng XZ trong frame `world`, tại `y = 0.221 m`.
  Đây là hướng tự nhiên của TCP ở tư thế home; dịch nhẹ ra ngoài robot và ít
  đổi nhánh IK hơn.
- Tâm chữ: `x = 0.12 m`; đáy chữ ở `z = 0.55 m`, cao `0.12 m`, nên toàn bộ
  nét nằm cao hơn mặt đất và tránh vùng thân robot.
- Kích thước chữ: cao `0.12 m`, rộng `0.08 m`; tỉ lệ này đủ rõ trong RViz/Gazebo
  và giữ cung cong cách xa vùng self-collision của robot.
- Waypoint: một nét liên tục gồm đoạn thẳng từ chân phải lên đỉnh phải,
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

### 2.1 Vì sao dùng MoveIt 2?

Tọa độ Cartesian của TCP không thể gửi trực tiếp đến joint controller. MoveIt 2
đảm nhiệm các bước trung gian:

1. nhận pose hoặc danh sách Cartesian waypoint;
2. giải bài toán inverse kinematics để tìm góc khớp;
3. kiểm tra giới hạn khớp và self-collision;
4. sinh `JointTrajectory` theo thời gian;
5. gửi trajectory đến `joint_trajectory_controller` để Gazebo thực thi.

Node chỉ cho phép thực thi khi Cartesian fraction đạt `1.0`. Nếu MoveIt chỉ
đi được một phần đường, trajectory bị từ chối để robot không viết dở dang.

### 2.2 Nguyên lý tạo chữ D

Chữ được vẽ trong frame `world`, trên mặt phẳng XZ với `y = 0.221 m` cố định:

```text
                 z
                 ↑
        đỉnh  ┌──┐
              │  )
              │  )  ← nửa elip
        đáy   └──┘
                 └────────→ x
```

TCP giữ nguyên orientation trong toàn bộ nét vẽ. Waypoint gồm một đoạn thẳng
từ chân phải lên đỉnh phải, sau đó là nửa elip bên trái quay về chân phải.
Không có đoạn nối chéo khi TCP đang hạ xuống nên đường đi đúng là một nét chữ
liên tục trong một mặt phẳng.

Với `center_x`, `bottom_z`, `height` và `width`, tọa độ chân phải được tính là:

```text
x_right  = center_x + width / 2
z_bottom = bottom_z
```

Cung cong được lấy mẫu theo góc `theta`:

```text
x(theta) = x_right - width * cos(theta)
z(theta) = bottom_z + height / 2
           + height / 2 * sin(theta)
```

Waypoint chỉ thay đổi X và Z; Y giữ nguyên bằng `plane_y`. Vì vậy TCP luôn nằm
trên cùng một mặt phẳng Cartesian trong khi MoveIt chuyển đổi đường đi đó
thành góc khớp.

### 2.3 Marker được cập nhật như thế nào?

Node publish `visualization_msgs/msg/Marker` kiểu `LINE_STRIP` lên
`/drawing_path`. Trong lúc trajectory chạy, node đọc TF của `tool0` và thêm vị
trí TCP thực tế vào marker. Do đó đường xanh xuất hiện dần theo chuyển động
thật của robot, thay vì hiện sẵn toàn bộ waypoint. RViz đã được cấu hình sẵn
topic này nên không cần thêm Marker thủ công.

## 3. Các tham số chính

| Tham số | Giá trị mặc định | Ý nghĩa |
|---|---:|---|
| `plane_y` | `0.221 m` | Vị trí mặt phẳng viết theo trục Y; đã dịch nhẹ ra ngoài robot. |
| `center_x` | `0.12 m` | Tọa độ X tại tâm chữ. |
| `bottom_z` | `0.55 m` | Độ cao đáy chữ, tránh mặt đất. |
| `letter_height` | `0.12 m` | Chiều cao chữ D. |
| `letter_width` | `0.08 m` | Chiều rộng chữ D. |
| `lift_distance` | `0.05 m` | Khoảng nâng TCP trước và sau khi viết. |
| `eef_step` | `0.005 m` | Bước lấy mẫu Cartesian của MoveIt, tương đương 5 mm. |
| `kCurveWaypointCount` | `35` | Hằng số nội bộ: số mẫu mô tả cung cong của chữ D. |
| `velocity_scaling` | `0.05` | Hệ số giới hạn vận tốc của MoveIt. |
| `acceleration_scaling` | `0.05` | Hệ số giới hạn gia tốc của MoveIt. |
| `jump_threshold` | `0.5 rad` | Ngưỡng kiểm tra riêng bước nhảy tuyệt đối giữa các mẫu khớp. |
| trajectory tolerance | `0.30 rad` | Biên sai số trajectory của controller trong Gazebo. |

`eef_step` nhỏ giúp đường Cartesian bám waypoint mượt hơn nhưng tạo nhiều mẫu
trajectory hơn. Giá trị 5 mm là lựa chọn cân bằng giữa độ chính xác và thời
gian thực thi. Tốc độ và gia tốc 0.05 giúp controller mô phỏng bám trajectory
ổn định hơn khi Gazebo GUI và RViz cùng chạy.

MoveIt được gọi với collision checking bật. `computeCartesianPath()` dùng
`jump_threshold = 0.0` để không loại nhầm mẫu đầu do heuristic tương đối;
code thực hiện một kiểm tra tuyệt đối riêng với ngưỡng `0.5 rad` và từ chối nét
D nếu bước lớn nhất vượt ngưỡng. Nghiệm IK của HOVER cũng được đưa về gần
trạng thái khớp hiện tại để tránh robot đi vòng thêm `2π` ở một khớp.

## 4. Cấu trúc package

```text
<repository>/
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

## 5. Phụ thuộc

Môi trường kiểm thử:

- Ubuntu 22.04 hoặc WSL2 Ubuntu 22.04 có hỗ trợ GUI.
- ROS 2 Humble Desktop.
- MoveIt 2.
- Gazebo Fortress / `ur_simulation_gz`.
- Các package UR Humble: `ur_description`, `ur_moveit_config` và
  `ur_simulation_gz`.

Cài các package UR theo tài liệu chính thức của Universal Robots. Với
simulation, Universal Robots ROS Driver không cần chạy robot thật.

## 6. Build

### Với repo local hiện tại

Repo của bài đang nằm tại `/home/kieu/HRI/ur3`. Build từ thư mục cha của
package:

```bash
cd /home/kieu/HRI
source /opt/ros/humble/setup.bash

rosdep install --from-paths ur3 \
  --ignore-src -r -y --rosdistro humble

colcon build \
  --base-paths ur3 \
  --packages-select ur3_draw_letter \
  --symlink-install

source install/setup.bash
```

Nếu clone repo vào `~/ros2_ws/src/ur3_draw_letter` thay vì dùng đường dẫn trên,
đổi `ur3` thành `src` và chạy lệnh build từ `~/ros2_ws`.

## 7. Chạy simulation

Chỉ chạy một launch của package trong một thời điểm:

```bash
cd /home/kieu/HRI
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_draw_letter draw_letter_d.launch.py
```

Lệnh trên mở Gazebo và RViz. Launch tự chờ hai controller ở trạng thái
`active`, gửi một goal giữ tư thế khởi động rồi mới khởi động node vẽ. Node vẽ
chỉ được khởi động sau khi controller recovery hoàn tất thành công, thay vì
dựa vào một delay dài cố định.

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

## 8. Runtime validation

Sau khi launch đang chạy, mở terminal khác:

```bash
cd /home/kieu/HRI/ur3
./scripts/validate_runtime.sh
```

Script in trạng thái ROS distro, controller, MoveIt, joint state, action và
`/drawing_path`, sau đó kết luận `[PASS]` hoặc `[FAIL]`.

## 9. Kiểm tra kết quả

Trong terminal khác:

```bash
source /opt/ros/humble/setup.bash
source /home/kieu/HRI/install/setup.bash

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

## 10. Tài liệu tham khảo

- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Universal Robots ROS 2 Gazebo Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation)
- [Universal Robots ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [MoveIt examples for Universal Robots](https://github.com/dominikbelter/ros2_ur_moveit_examples)

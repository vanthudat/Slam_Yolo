# TurtleBot 4: Localization, Autonomous Navigation và YOLO

Hướng dẫn cho bài thực hành ROS 2 Humble: nạp bản đồ đã quét, định vị TurtleBot 4 bằng AMCL, tự đi qua các goal bằng Nav2, và nhận diện ảnh RGB bằng YOLO.

## 1. Yêu cầu và cài đặt

- Ubuntu 22.04, ROS 2 Humble Desktop.
- TurtleBot 4 Simulation chạy được trong Gazebo.
- Bản đồ maze đã quét gồm `maps/map_new.yaml` và `maps/map_new.pgm`.

Cài các gói ROS 2 nếu máy chưa có:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-navigation \
  ros-humble-nav2-simple-commander \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-cv-bridge \
  python3-colcon-common-extensions python3-pip
```

Cài YOLO và OpenCV cho Python 3.10 của Ubuntu. Các giới hạn phiên bản giúp tránh NumPy 2 và setuptools 80+ xung đột với `cv_bridge`/`colcon` trong môi trường ROS Humble:

```bash
python3 -m pip install --user \
  'setuptools<80' 'numpy<2' 'opencv-python>=4.7,<4.12' ultralytics
```

Kiểm tra thư viện:

```bash
python3 -c 'import cv2, numpy, ultralytics; from cv_bridge import CvBridge; print("OpenCV", cv2.__version__, "NumPy", numpy.__version__, "YOLO", ultralytics.__version__)'
```

Ultralytics có thể tải thêm PyTorch và các gói phụ thuộc ở lần cài đầu. Tài liệu: [TurtleBot 4 Simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html), [Nav2 Simple Commander](https://docs.nav2.org/commander_api/index.html), [Ultralytics](https://docs.ultralytics.com/).

## 2. Build package

Mở terminal tại workspace:

```bash
cd /home/vtd/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_sim --symlink-install
source /home/vtd/ros2_ws/install/setup.bash
```

Package có các overlay cho `irobot_create_description`, `irobot_create_control` và `turtlebot4_description`, bao gồm cấu hình renderer `ogre2` cho GPU LiDAR của Gazebo Fortress. Vì vậy, cần build và source workspace này trước khi chạy.

Mỗi terminal ROS mới dùng trong bài cần nạp môi trường và đặt cùng domain:

```bash
source /opt/ros/humble/setup.bash
source /home/vtd/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
```

Các launch của dự án cũng đặt domain 24 cho các node mà chúng khởi chạy. ROS CLI chạy ở terminal khác vẫn cần `ROS_DOMAIN_ID=24`.

## 3. Bản đồ và Localization

Bản đồ sử dụng là `maps/map_new.yaml` cùng ảnh `maps/map_new.pgm`. Launch navigation mặc định nạp map này; khi chạy lệnh bên dưới vẫn truyền đường dẫn tường minh để tránh chọn nhầm map cũ.

Chạy Gazebo, các vật thể của maze, AMCL, Nav2 và RViz2 trong **một terminal**:

```bash
ros2 launch tb4_sim navigation.launch.py world:=maze model:=standard \
  maze_obstacle:=true \
  map:=/home/vtd/ros2_ws/src/tb4_sim/maps/map_new.yaml
```

Không chạy đồng thời `localization.launch.py` hoặc `sim.launch.py`; `navigation.launch.py` đã khởi chạy các thành phần cần cho localization và navigation. Chờ Nav2 active rồi kiểm tra:

```bash
ros2 topic info /scan
ros2 topic info /amcl_pose
ros2 topic info /map
```

Trên RViz2, xác nhận map hiển thị đúng, robot khớp vị trí trong Gazebo, và tia `/scan` trùng với tường. Nếu pose ban đầu không khớp, dùng **2D Pose Estimate** trong RViz2 trước khi gửi goal.

### Vùng tránh ghế dài

Để robot không chui qua hai chân đỡ của ghế dài ở `(-3, 1)`, launch nạp `maps/bench_keepout.yaml` vào Keepout Filter cho cả global và local costmap. Mask là lớp tránh riêng; `map_new.pgm` gốc không bị sửa. Trên RViz2 bật **Global Costmap**, **Local Costmap** và **Path** để kiểm tra planner vòng ngoài vùng ghế. Các topic của filter:

```bash
ros2 topic info /bench_keepout_info
ros2 topic info /bench_keepout_mask
```

Nếu sửa các file YAML hoặc mask, build lại package rồi khởi chạy lại navigation để Nav2 nạp cấu hình mới.

## 4. Xem camera RGB và chạy YOLO

Launch navigation đã mở RViz2 bằng cấu hình có sẵn Image display cho `/yolo/annotated_image`. Trong terminal khác, kiểm tra ảnh camera:

```bash
ros2 topic list | grep -E 'image_raw|scan|amcl_pose|map'
ros2 topic hz /oakd/rgb/preview/image_raw
```

Chạy node YOLO:

```bash
ros2 run tb4_sim yolo_detector --ros-args -p use_sim_time:=true
```

Node subscribe `/oakd/rgb/preview/image_raw`, chuyển ảnh qua `cv_bridge`, chạy YOLO, vẽ bounding box và confidence, publish `/yolo/annotated_image`, đồng thời ghi nhãn phát hiện ra terminal. RViz2 sẽ hiển thị ảnh đã chú thích qua display có sẵn.

Mặc định node dùng YOLO-World (`yolov8s-world.pt`) với prompt riêng cho `tree`, `table`, `chair`, `bench`, `person`, `suitcase`, `barrel` và hình `red cylinder`. Overlay và log dùng nhãn tiếng Anh ASCII để tránh lỗi font OpenCV. Vì bàn, ghế, vali và bình cứu hỏa trong maze là các SDF low-poly, node có thêm OpenCV color/shape fallback dành riêng cho các mô hình này: nhận diện mặt gỗ dài thành `bench`, mặt bàn thành `table`, và lưng ghế thành `chair`, nhận diện vali xanh đậm, thùng phuy xanh-teal, bình cứu hỏa đỏ, và thân xanh của mannequin khi nhìn thấy. Các fallback theo màu chỉ áp dụng cho asset Gazebo hiện tại, không thay thế model YOLO tổng quát. Node dùng confidence `0.03` và ảnh suy luận `960`; ngưỡng thấp giúp hiện box yếu hơn nhưng làm tăng dự đoán nhầm. Lần chạy đầu cần tải trọng số YOLO-World và bộ mã hóa CLIP.

Cấu hình prompt mở tăng cơ hội nhận ra cây và bàn mô phỏng, nhưng vẫn có thể nhầm vì các mô hình Gazebo được dựng từ hình học đơn giản. Nếu nhãn không xuất hiện, kiểm tra camera có nhìn rõ vật thể và `/oakd/rgb/preview/image_raw` có ảnh. Có thể đổi trọng số bằng `-p model_path:=/duong/dan/model.pt`; các prompt đang dùng cấu hình qua tham số `class_allowlist`.

YOLO-World có thể nhận prompt theo tên lớp, nhưng kết quả trên mô hình mô phỏng vẫn cần kiểm tra thực tế. Muốn nhận diện ổn định các vật thể tùy biến như bình chữa cháy hoặc thùng xăng/thùng phuy, cần thu ảnh và huấn luyện/tinh chỉnh model riêng.

## 5. Goal và điều hướng tự động

Bốn goal mẫu được lưu trong `config/route_goals.yaml`, theo thứ tự Goal 1 → Goal 2 → Goal 3 → Goal 4:

| Goal | Vị trí | x (m) | y (m) | yaw (rad) |
|---|---|---:|---:|---:|
| 1 | Khu vực trước cây phía tây bắc | -6.9 | 6.9 | 2.4 |
| 2 | Gần người | -6.4 | -2.1 | -1.2 |
| 3 | Khu vực bình chữa cháy | 4.0 | -3.0 | 0.0 |
| 4 | Trước ghế dài | -3.0 | 0.0 | 1.5708 |

Để lấy tọa độ trên map, chọn **Publish Point** trên RViz2 rồi xem:

```bash
ros2 topic echo /clicked_point
```

`/clicked_point` cho vị trí `x, y`; đặt yaw theo hướng robot cần quay khi tới goal. Chỉnh các giá trị `goalN_x`, `goalN_y`, `goalN_yaw` và `goal_count` trong `config/route_goals.yaml` nếu dùng tọa độ khác.

Sau khi AMCL và Nav2 active, chạy node điều hướng trong terminal khác:

```bash
ros2 run tb4_sim route_navigator --ros-args \
  --params-file /home/vtd/ros2_ws/src/tb4_sim/config/route_goals.yaml
```

Node đợi AMCL/Nav2 active, tự undock nếu cần, rồi gửi từng goal bằng Nav2 và chỉ chuyển goal khi goal trước thành công. Trong lúc node chạy, không dùng Teleop và không đặt goal bằng chuột trên RViz2.

## 6. Tạo lại bản đồ (chỉ khi cần quét lại)

Nếu cần quét lại maze, dừng phiên navigation trước, rồi mở mapping:

```bash
ros2 launch tb4_sim mapping.launch.py world:=maze model:=standard maze_obstacle:=true
```

Quét các hành lang bằng Teleop, theo dõi map trên RViz2, rồi lưu bản đồ từ terminal khác:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/vtd/ros2_ws/src/tb4_sim/maps/map_new
```

Lệnh tạo/cập nhật cặp `map_new.yaml` và `map_new.pgm`. Sau đó dừng mapping, kiểm tra bản đồ, rồi khởi chạy lại navigation ở Phần 3.

## Cấu hình liên quan

- `config/localization.yaml`: tham số AMCL và pose khởi tạo mặc định `(0, 0, 0)`.
- `config/nav2_keepout.yaml`, `config/keepout_filter.yaml`: Nav2 costmap và vùng tránh ghế dài.
- `config/route_goals.yaml`: bốn goal điều hướng mẫu.
- `config/slam.yaml`: tham số SLAM Toolbox.
- `launch/maze_obstacle.launch.py`, `models/`: vật thể mô phỏng và vị trí spawn.
- `rviz/navigation_yolo.rviz`: hiển thị map, costmap, Path và ảnh YOLO.
- `maps/map.yaml`, `maps/map.pgm`: map cũ; `maps/map_new.yaml`, `maps/map_new.pgm`: map hiện dùng.

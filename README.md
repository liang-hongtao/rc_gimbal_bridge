# rc_gimbal_bridge 使用说明与工作原理

## 快速上手（每个 launch 的最简用法）

- 单路 RTSP → ROS 话题
  - 命令：`roslaunch rc_gimbal_bridge rtsp_camera_relay.launch video_stream_url:=rtsp://<IP>:<port>/stream=1`
  - 功能：拉取一条 RTSP 流，发布 `/rtsp_camera_relay/image`、`/rtsp_camera_relay/camera_info`、`/rtsp_camera_relay/status`。

- 双路 RTSP → ROS 话题（rgb + thermal）
  - 命令：`roslaunch rc_gimbal_bridge rtsp_dual_camera_relay.launch rgb_url:=rtsp://<IP>:554/stream=1 thermal_url:=rtsp://<IP>:555/stream=2`
  - 功能：同时发布 `/rgb/image` 与 `/thermal/image` 及各自的 `camera_info/status`。

- 两路取流 + RTSP 服务（供 QGC/播放器取流）
  - 命令：`roslaunch rc_gimbal_bridge rtsp_dual_to_qgc.launch`
  - 功能：自动拉取两路 RTSP 并转为 ROS 话题，同时在本机 8554 端口提供 RTSP：`rtsp://<Jetson_IP>:8554/rgb` 与 `/thermal`。

- 仅 RTSP 服务（把已有 ROS 图像话题推为 RTSP）
  - 命令：`roslaunch rc_gimbal_bridge rtsp_topics_server.launch`
  - 功能：读取 `config/rtsp_topics.yaml` 中配置的多路话题，推流到 `rtsp://<Jetson_IP>:8554/<mount>`。

- RC 通道监视（MAVROS）
  - 命令：`roslaunch rc_gimbal_bridge rc_channel_monitor.launch`
  - 功能：监视 `/mavros/rc/in`，按阈值打印变化并在退出时输出各通道最小/最大值。

- RC → UDP 云台控制
  - 命令：`roslaunch rc_gimbal_bridge rc_to_udp_gimbal.launch`
  - 功能：将 CH9/CH10 映射为偏航/俯仰速度，通过 UDP 向相机/云台发送控制指令。

- 双路 RTSP + RC 云台控制 + PX4 + 键盘控制（集成启动）
  - 命令：`roslaunch rc_gimbal_bridge rc_dual_stream_with_gimbal.launch`
  - 功能：同时发布 `/rgb/image` 与 `/thermal/image`，启动 RC→UDP 云台控制与键盘控制，并联动启动 `robot_bringup/px4.launch`（MAVROS/飞控）。
  - 可选参数：
    - `use_keyboard_gui:=true|false`（默认 true）开启/关闭带窗口的键盘控制。
    - `use_keyboard_console:=true|false`（默认 false）如需控制台版本并在 xterm 中捕获键盘。

提示：若使用本包内置 RTSP 服务（rtsp_dual_to_qgc.launch / rtsp_topics_server.launch），需要系统安装 GStreamer 运行期组件与 Python GI，详见下文依赖。

本包提供如下功能：
- RTSP 实时视频拉流并发布为 ROS 图像话题（单路/双路）。
- 将 ROS 图像话题再推送为 RTSP 服务，便于 QGC/地面站/播放器取流。
- 监视遥控器 RC 通道变化（MAVROS）。
- 将 RC 通道映射为 UDP 指令控制云台（偏航/俯仰速度或位置）。
- 一个独立的终端键盘控制示例（非 ROS）。

适配环境：Ubuntu 20.04，ROS Noetic。GPU/YOLO 不在本包内，本文不涉及。


**依赖**
- ROS: `roscpp`、`rospy`、`cv_bridge`、`image_transport`、`sensor_msgs`、`std_msgs`、`mavros_msgs`
- OpenCV（C++ 节点使用 `cv::VideoCapture`）
- 若使用本包自带 RTSP 服务（Python）：
  - 运行期需安装 GStreamer 与 Python GI 绑定：
    - `sudo apt-get install -y python3-gi gir1.2-gst-rtsp-server-1.0 \
        gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-libav`


**编译**
- `cd ~/cwkj_ws && catkin_make`
- `source ~/cwkj_ws/devel/setup.bash`


## 包内主要文件与功能

- C++ RTSP 拉流节点（将 RTSP → ROS 图像话题）
  - 头文件：`include/rc_gimbal_bridge/rtsp_camera_relay.hpp:1`
  - 实现：`src/rtsp_camera_relay.cpp:1`
  - 主程序：`src/rtsp_camera_relay_main.cpp:1`
- Python RTSP 服务（将 ROS 图像话题 → RTSP）
  - 节点脚本：`scripts/rtsp_image_server.py:1`
  - 默认配置：`config/rtsp_topics.yaml:1`
- 键盘控制（GUI）：`scripts/gimbal_keyboard_gui.py:1`
- RC 监视与控制
  - RC 通道监视：`scripts/rc_channel_monitor.py:1`
  - RC → UDP 云台控制：`scripts/rc_to_udp_gimbal.py:1`
  - 终端键控（示例/非 ROS）：`scripts/realtime_control_v2.py:1`
- Launch 启动
  - 单路 RTSP 拉流：`launch/rtsp_camera_relay.launch:1`
  - 双路 RTSP 拉流（rgb/thermal）：`launch/rtsp_dual_camera_relay.launch:1`
  - 双路取流 + RTSP 服务（供 QGC）：`launch/rtsp_dual_to_qgc.launch:1`
  - 仅 RTSP 服务（从 ROS 话题推流）：`launch/rtsp_topics_server.launch:1`
  - RC 监视：`launch/rc_channel_monitor.launch:1`
  - RC → UDP 云台：`launch/rc_to_udp_gimbal.launch:1`
  - 集成：双路 RTSP + RC 云台 + PX4 + 键盘：`launch/rc_dual_stream_with_gimbal.launch:1`

注意：键盘控制脚本需要交互式终端。如果通过 roslaunch 无法捕获按键，请在另一个终端直接运行：
- `rosrun rc_gimbal_bridge realtime_control_v2.py`


## 使用示例

1) 单路 RTSP → ROS 图像话题
- 命令：
  - `roslaunch rc_gimbal_bridge rtsp_camera_relay.launch video_stream_url:=rtsp://<IP>:<port>/stream=1`
- 话题（私有命名空间下）：
  - `~image`（sensor_msgs/Image）、`~camera_info`、`~status`（std_msgs/String）
- 可在 rqt_image_view 选择 `/rtsp_camera_relay/image` 查看。

2) 双路 RTSP → ROS（rgb + thermal）
- 命令：
  - `roslaunch rc_gimbal_bridge rtsp_dual_camera_relay.launch \
      rgb_url:=rtsp://<IP>:554/stream=1 thermal_url:=rtsp://<IP>:555/stream=2`
- 话题：`/rgb/image` 与 `/thermal/image`（以及各自 camera_info/status）。

3) 将两路 ROS 图像对外提供 RTSP（供 QGC/播放器）
- 一键启动（先拉流再推流）：
  - `roslaunch rc_gimbal_bridge rtsp_dual_to_qgc.launch`
- 在 QGC/播放器填写：
  - 可见光：`rtsp://<Jetson_IP>:8554/rgb`
  - 热成像：`rtsp://<Jetson_IP>:8554/thermal`
- 注意：手机上的 QGC 必须与 Jetson 处于可互访的同一 IP 网络（例如手机热点，Jetson 连接该热点后使用其 `wlan0` IP）。

4) 仅 RTSP 服务（从已存在的 ROS 图像话题推流）
- 命令：`roslaunch rc_gimbal_bridge rtsp_topics_server.launch`
- 默认读取 `config/rtsp_topics.yaml:1` 中的端口与流列表。

5) RC 工具
- 监视 RC 通道：
  - `roslaunch rc_gimbal_bridge rc_channel_monitor.launch`
- RC → UDP 云台控制：
  - `roslaunch rc_gimbal_bridge rc_to_udp_gimbal.launch`


## 参数与用法详解

### A. C++ 节点：rtsp_camera_relay_node
用途：通过 OpenCV 从 RTSP 拉流，发布 ROS 图像话题。

主要参数（私有命名空间）：
- `~video_stream_url`：RTSP 地址（例如 `rtsp://192.168.144.108:554/stream=1`）。
- `~backend`：`ffmpeg`、`gstreamer` 或 `any`。默认 `ffmpeg`。
- `~rtsp_transport`：`tcp` 或 `udp`。默认空（不指定）。我们在 launch 中默认设为 `tcp`。
- `~timeout_ms`：超时毫秒（映射为 FFmpeg 的 `stimeout` 微秒）。
- `~ffmpeg_low_latency`：是否启用低时延选项（`fflags=nobuffer|flags=low_delay|framedrop=1|max_delay=0|reorder_queue_size=0`）。
- `~ffmpeg_capture_options`：覆盖字符串（格式 `key;val|key;val`，若设置将替代上面几个自动拼的选项）。

工作原理概述：
- 代码位置：`src/rtsp_camera_relay.cpp:1`。
- 构建并设置 `OPENCV_FFMPEG_CAPTURE_OPTIONS` 环境变量，优先尝试 `cv::CAP_FFMPEG` 打开，失败回退到 GStreamer，再回退到 CAP_ANY。
- 循环 `VideoCapture::read` 获取帧，封装为 `sensor_msgs/Image`（BGR8），并发布 `~image`；同时发布 `~camera_info` 与 `~status`（`"live"` or `"No frame from camera"`）。

常见日志：
- 启动初期可能看到 HEVC 提示 “Could not find ref with POC …”，通常在拿到关键帧后恢复，属于非致命。


### B. Python 节点：rtsp_image_server.py
用途：将 ROS 图像话题转为本地 RTSP 服务（单端口，多个 mountpoint）。

启动方式：
- 直接：`rosrun rc_gimbal_bridge rtsp_image_server.py _port:=8554 _streams:='{rgb: {topic: /rgb/image, mountpoint: /rgb}}'`
- 推荐：`roslaunch rc_gimbal_bridge rtsp_topics_server.launch`（从配置文件加载参数）。

参数：
- `~port`（或全局 `port`）：RTSP 服务端口，默认 8554。
- `~streams`（或全局 `streams`）：字典，定义多路流。每路支持键：
  - `topic`（或 `source`）：ROS 图像话题名，例如 `/rgb/image`。
  - `mountpoint`：RTSP 路径，例如 `/rgb`。
  - `framerate`：帧率（整数，默认 20）。
  - `bitrate`：码率（单位 kbps，默认 1500）。
  - `width`/`height`：可选；若未设置，以收到的第一帧为准。
  - `format`：appsrc caps 的像素格式（默认 `BGR`）。

工作原理概述：
- 代码位置：`scripts/rtsp_image_server.py:1`。
- 订阅每路 `sensor_msgs/Image`，转换为 BGR，首次收到帧时按实际分辨率创建 mount：
  - GStreamer 管线：`appsrc → videoconvert → x264enc (zerolatency, ultrafast) → rtph264pay → RTSP`。
  - `appsrc` 以 `do-timestamp=true` 推送，以 `framerate` 节流。
- 每一路独立 `appsrc`，RTSP 服务由 `GstRtspServer` 提供。默认 CPU x264 编码（如需硬编，可以扩展为 `nvv4l2h264enc`）。

注意：浏览器普遍不支持 RTSP，建议用 QGC、VLC、ffplay 验证。


### C. RC 工具

1) RC 通道监视（`scripts/rc_channel_monitor.py:1`）
- 订阅 `/mavros/rc/in`，周期打印变化超过阈值的通道，并在退出时输出各通道的最小/最大值。
- 参数：
  - `~change_thresh`（默认 25us）、`~print_every`（默认 0.5s）、`~show_all`、`~max_channels`。
- 启动：`roslaunch rc_gimbal_bridge rc_channel_monitor.launch`

2) RC → UDP 云台控制（`scripts/rc_to_udp_gimbal.py:1`）
- 将 RC 通道（默认 CH9=偏航、CH10=俯仰）映射为速度或位置命令，通过 UDP 向相机/云台发送厂商协议报文（带 1 字节校验和）。
- 参数要点：
  - 网络：`~camera_ip`（默认 192.168.144.108）、`~port`（默认 5000）。
  - 通道映射：`~ch_yaw`、`~ch_pitch`（1 基）。
  - 端点/中心/死区：`~pwm_min_*`、`~pwm_max_*`、`~center_*`、`~deadband_*`。
  - 速度限制/频率：`~max_yaw`、`~max_pitch`、`~repeat_hz`。
  - 模式：`~control_mode`=`deadband_speed`（推荐）/`speed`/`position`。
  - 方向：`~invert_yaw`、`~invert_pitch`。
- 工作原理（简述）：
  - `deadband_speed`：中心±死区内速度为 0，越离中心越快，端点达最大设定速度。
  - `speed`：以 1500 为中心线性映射到速度。
  - `position`：PWM 线性映射到目标角度，用比例系数（`kp_*`）生成速度命令并限幅。
  - 失效保护：PWM 异常（<900 或 >2100）时发送停止命令。
- 启动：`roslaunch rc_gimbal_bridge rc_to_udp_gimbal.launch`

3) 键盘控制
- GUI 版本（推荐，支持 roslaunch 启动）：`rosrun rc_gimbal_bridge gimbal_keyboard_gui.py`，或通过集成 launch 自动弹窗。
- 终端键控示例（`scripts/realtime_control_v2.py:1`，非 ROS）：
  - 通过 UDP 控制云台：WASD 方向、空格停止、C 回中、+/- 变倍、1~4 指定倍数。
  - 修改脚本顶部 `CAMERA_IP` 与 `UDP_PORT` 后运行：`python3 scripts/realtime_control_v2.py`。
  - 如需在 roslaunch 中使用控制台版本，可设置：`use_keyboard_console:=true`（需系统安装 `xterm`）。


## RTSP/QGC 网络说明与建议

- QGC 与 Jetson 必须在同一可互访的 IP 网络（例如：手机热点，Jetson 连接该热点）。
- QGC URL 示例：
  - `rtsp://<Jetson_IP>:8554/rgb`
  - `rtsp://<Jetson_IP>:8554/thermal`
- Jetson 上常见网卡 IP：
  - `eth0`（有线）：例如 `192.168.144.x`
  - `wlan0`（Wi‑Fi/热点）：例如 `192.168.1.x`
- 自检：
  - Jetson：`hostname -I` 查看 IP；`ss -ltnp | grep 8554` 查看 RTSP 端口监听。
  - 本机播放：`vlc rtsp://127.0.0.1:8554/rgb`。


## 性能与延迟调优

- 拉流端（C++）
  - `rtsp_transport:=tcp` 与 `ffmpeg_low_latency:=true` 可提高稳定性与降低缓冲延迟。
  - 相机端（若可配）缩短关键帧间隔（GOP），打开周期性 IDR，可减轻启动花屏/延迟。

- 推流端（Python）
  - 默认 `x264enc tune=zerolatency speed-preset=ultrafast`，适合低延迟。
  - 可在 `config/rtsp_topics.yaml:1` 降低 `framerate`、分辨率或 `bitrate` 以减少延迟与卡顿。
  - 客户端（VLC/QGC）适当降低网络缓存也很关键。


## 常见问题排查

- 浏览器打不开 RTSP（或延迟极高）：多数浏览器不支持 RTSP，请改用 VLC/ffplay/QGC。
- 只有一路有画面：检查是使用了本包的 RTSP 服务（`rtsp_image_server.py`），而非旧的 `camera_rtsp`；两路 `streams` 都应为基于 topic 的配置。
- hevc “Could not find ref with POC …”：启动初期常见，收到关键帧后会恢复，一般无需处理。
- RTSP 无法连接：检查防火墙、端口 8554、Jetson 与客户端是否同网并能互相 `ping` 通。


## 许可与来源

- `rtsp_camera_relay` 参考自 rocon_rtsp_camera_relay（BSD 许可），在本包内作了最小改动以适配参数与低延迟选项。
- 其余脚本为本包定制实现。

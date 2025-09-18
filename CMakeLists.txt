cmake_minimum_required(VERSION 3.0.2)
project(rc_gimbal_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  mavros_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rtsp_camera_relay_node
  src/rtsp_camera_relay_main.cpp
  src/rtsp_camera_relay.cpp
)
target_link_libraries(rtsp_camera_relay_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS rtsp_camera_relay_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/rc_to_udp_gimbal.py
  scripts/rc_channel_monitor.py
  scripts/rtsp_image_server.py
  scripts/realtime_control_v2.py
  scripts/gimbal_keyboard_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

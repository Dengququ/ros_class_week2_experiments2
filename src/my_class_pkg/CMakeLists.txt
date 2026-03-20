cmake_minimum_required(VERSION 3.0.2)
project(my_class_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
  actionlib
  actionlib_msgs
)

## 自定义消息
add_message_files(
  DIRECTORY msg
  FILES MyMessage.msg
)

## 自定义服务
add_service_files(
  DIRECTORY srv
  FILES MyServiceMsg.srv
)

## 自定义动作
add_action_files(
  DIRECTORY action
  FILES MyAction.action
)

generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime dynamic_reconfigure actionlib actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 5.1.1 C++ 标准消息发布者
add_executable(ros_publisher_node src/ros_publisher.cpp)
target_link_libraries(ros_publisher_node ${catkin_LIBRARIES})
add_dependencies(ros_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 5.1.2 C++ 标准消息订阅者
add_executable(ros_subscriber_node src/ros_subscriber.cpp)
target_link_libraries(ros_subscriber_node ${catkin_LIBRARIES})
add_dependencies(ros_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 5.3.1 C++ 自定义消息发布者
add_executable(msg_publisher_node src/msg_publisher.cpp)
target_link_libraries(msg_publisher_node ${catkin_LIBRARIES})
add_dependencies(msg_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 5.3.2 C++ 自定义消息订阅者
add_executable(msg_subscriber_node src/msg_subscriber.cpp)
target_link_libraries(msg_subscriber_node ${catkin_LIBRARIES})
add_dependencies(msg_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 5.2 C++ 服务端
add_executable(ros_server_node src/ros_server.cpp)
target_link_libraries(ros_server_node ${catkin_LIBRARIES})
add_dependencies(ros_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 5.2 C++ 客户端
add_executable(ros_client_node src/ros_client.cpp)
target_link_libraries(ros_client_node ${catkin_LIBRARIES})
add_dependencies(ros_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 5.2 C++ 动作服务端
add_executable(ros_action_server src/ros_action_server.cpp)
target_link_libraries(ros_action_server ${catkin_LIBRARIES})
add_dependencies(ros_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 5.2 C++ 动作客户端
add_executable(ros_action_client src/ros_action_client.cpp)
target_link_libraries(ros_action_client ${catkin_LIBRARIES})
add_dependencies(ros_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 传感器实验 — 碰撞传感器订阅
add_executable(ros_bump_node src/ros_bump.cpp)
target_link_libraries(ros_bump_node ${catkin_LIBRARIES})
add_dependencies(ros_bump_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 传感器实验 — 碰撞传感器避障
add_executable(ros_bump_avoid_node src/ros_bump_avoid.cpp)
target_link_libraries(ros_bump_avoid_node ${catkin_LIBRARIES})
add_dependencies(ros_bump_avoid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 传感器实验 — 超声波订阅
add_executable(ros_sonic_node src/ros_sonic.cpp)
target_link_libraries(ros_sonic_node ${catkin_LIBRARIES})
add_dependencies(ros_sonic_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 传感器实验 — TOF 避障
add_executable(ros_tof_avoid_node src/ros_tof_avoid.cpp)
target_link_libraries(ros_tof_avoid_node ${catkin_LIBRARIES})
add_dependencies(ros_tof_avoid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 传感器实验 — IMU 订阅
add_executable(ros_imu_node src/ros_imu.cpp)
target_link_libraries(ros_imu_node ${catkin_LIBRARIES})
add_dependencies(ros_imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 传感器实验 — IMU 自旋 180°
add_executable(ros_imu_spin_node src/ros_imu_spin.cpp)
target_link_libraries(ros_imu_spin_node ${catkin_LIBRARIES})
add_dependencies(ros_imu_spin_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 参数实验 - C++ 参数读写
add_executable(ros_param src/ros_param.cpp)
target_link_libraries(ros_param ${catkin_LIBRARIES})
add_dependencies(ros_param ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 动态参数实验 - 基础动态参数服务端
add_executable(dynamic_reconfigure_node src/dynamic_reconfigure.cpp)
add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_reconfigure_node ${catkin_LIBRARIES})

## 动态参数实验 - 小乌龟速度动态配置
add_executable(ros_dynamic_speed_node src/ros_dynamic_speed.cpp)
add_dependencies(ros_dynamic_speed_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_dynamic_speed_node ${catkin_LIBRARIES})

## 日志实验 - C++ 日志节点
add_executable(ros_log src/ros_log.cpp)
target_link_libraries(ros_log ${catkin_LIBRARIES})
add_dependencies(ros_log ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 3.0.2)
project(control_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

# 声明消息文件
add_message_files(
  FILES
  TimestampedFloat32MultiArray.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

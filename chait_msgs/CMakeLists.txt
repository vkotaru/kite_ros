cmake_minimum_required(VERSION 3.0.2)
project(chait_msgs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs actionlib_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  QCommand.msg
  QCommandStamped.msg
  Spline.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs actionlib_msgs)

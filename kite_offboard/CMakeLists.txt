cmake_minimum_required(VERSION 3.0.2)
project(kite_offboard)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        rospy
        std_msgs
        kite_msgs
        )


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES qrotor_offboard
        #  CATKIN_DEPENDS geometry_msgs nav_msgs qrotor_gazebo rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

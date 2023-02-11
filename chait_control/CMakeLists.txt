cmake_minimum_required(VERSION 3.0.2)
project(chait_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES chait_control
#        CATKIN_DEPENDS chait_control
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_library(chait_control INTERFACE include)
target_include_directories(chait_control INTERFACE include)


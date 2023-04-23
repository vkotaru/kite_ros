cmake_minimum_required(VERSION 3.0.2)
project(kite_control)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES kite_control
#        CATKIN_DEPENDS kite_control
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_library(kite_control INTERFACE)
target_include_directories(kite_control INTERFACE)


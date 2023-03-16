cmake_minimum_required(VERSION 3.0.2)
project(chait_companions)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        chait_control
        chait_msgs
        geometry_msgs
        nav_msgs
        roscpp
        std_msgs
        )

catkin_package(
        INCLUDE_DIRS include

        LIBRARIES chait_companions
        CATKIN_DEPENDS chait_msgs geometry_msgs nav_msgs roscpp std_msgs chait_control

        # DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(SRCS
        src/drone_manager.cpp
        src/drone_companion.cpp)

add_library(chait_companions ${SRCS})
add_dependencies(chait_companions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chait_companions ${catkin_LIBRARIES})



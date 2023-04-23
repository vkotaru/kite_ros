cmake_minimum_required(VERSION 3.0.2)
project(kite_gazebo_plugins)
find_package(gazebo)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        std_msgs
        kite_msgs

        kite_control

        ddynamic_reconfigure
        tf2
        tf2_ros
        tf2_geometry_msgs
        eigen_conversions

        gazebo_plugins
        gazebo_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES kite_gazebo_plugins
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs gazebo_plugins gazebo_ros
        DEPENDS EIGEN3 GAZEBO
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(kite_GAZEBO_SRCS
        src/kite_gazebo_plugin.cpp)


add_library(kite_gazebo_plugins SHARED ${kite_GAZEBO_SRCS})
target_link_libraries(kite_gazebo_plugins ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(kite_gazebo_plugins ${catkin_EXPORTED_TARGETS})

add_library(kite_quadrotor_plugin SHARED src/kite_quadrotor_plugin.cpp)
target_link_libraries(kite_quadrotor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} kite_gazebo_plugins)
add_dependencies(kite_quadrotor_plugin ${catkin_EXPORTED_TARGETS})
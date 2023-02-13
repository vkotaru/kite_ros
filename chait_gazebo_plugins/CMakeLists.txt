cmake_minimum_required(VERSION 3.0.2)
project(chait_gazebo_plugins)
find_package(gazebo)


add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        std_msgs
        chait_msgs

        chait_control

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
        LIBRARIES chait_gazebo_plugins
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

set(CHAIT_GAZEBO_SRCS
        src/chait_gazebo_plugin.cpp)


add_library(chait_gazebo_plugins SHARED ${CHAIT_GAZEBO_SRCS})
target_link_libraries(chait_gazebo_plugins ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(chait_gazebo_plugins ${catkin_EXPORTED_TARGETS})

add_library(chait_quadrotor_plugin SHARED src/chait_quadrotor_plugin.cpp)
target_link_libraries(chait_quadrotor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} chait_gazebo_plugins)
add_dependencies(chait_quadrotor_plugin ${catkin_EXPORTED_TARGETS})
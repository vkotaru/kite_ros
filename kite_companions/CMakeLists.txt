cmake_minimum_required(VERSION 3.0.2)
project(kite_companions)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        kite_control
        kite_msgs
        geometry_msgs
        nav_msgs
        roscpp
        std_msgs
        eigen_conversions
        ddynamic_reconfigure
        )

catkin_package(
        INCLUDE_DIRS include

        LIBRARIES kite_companions
        CATKIN_DEPENDS kite_msgs geometry_msgs nav_msgs roscpp std_msgs kite_control

        # DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(SRCS
        src/drone_manager.cpp
        src/drone_companion.cpp)

# Libraries
#############
add_library(kite_companions ${SRCS})
add_dependencies(kite_companions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kite_companions ${catkin_LIBRARIES})

# Executables
#############
add_executable(drone_companion_node src/nodes/drone_companion_node.cpp)
add_dependencies(drone_companion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_companion_node ${catkin_LIBRARIES} kite_companions)

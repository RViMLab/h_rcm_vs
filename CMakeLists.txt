cmake_minimum_required(VERSION 3.0.2)
project(h_rcom_vs)

# Catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_ros_planning_interface
    geometry_msgs
    actionlib
    tf
    tf_conversions
    rcom_impl
    rcom_msgs
)

# Other packages
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp moveit_ros_planning_interface geometry_msgs actionlib tf tf_conversions rcom_impl rcom_msgs
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
    DEPENDS EIGEN3
)

# Includes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Library
add_library(h_rcom_vs src/h_rcom_vs_action_client.cpp)
target_link_libraries(h_rcom_vs ${catkin_LIBRARIES})

# Executable
add_executable(h_rcom_vs_client_node src/h_rcom_vs_client_node.cpp)
target_link_libraries(h_rcom_vs_client_node ${catkin_LIBRARIES} h_rcom_vs)

add_executable(h_rcom_vs_server_node src/h_rcom_vs_server_node.cpp)
target_link_libraries(h_rcom_vs_server_node ${catkin_LIBRARIES})

add_executable(rcom_init_node src/rcom_init_node.cpp)
target_link_libraries(rcom_init_node ${catkin_LIBRARIES})

# Install
install(
    TARGETS h_rcom_vs
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
    DIRECTORY include/h_rcom_vs/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
    TARGETS h_rcom_vs_client_node h_rcom_vs_server_node rcom_init_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

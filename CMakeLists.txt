cmake_minimum_required(VERSION 3.0.2)
project(h_rcm_vs)

# Catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge 
    image_transport 
    image_geometry
    moveit_ros_planning_interface
    geometry_msgs
    actionlib
    tf
    tf_conversions
    rcm_impl
    rcm_msgs
)

# Other packages
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp cv_bridge image_transport image_geometry moveit_ros_planning_interface geometry_msgs actionlib tf tf_conversions rcm_impl rcm_msgs
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
add_library(h_rcm_vs src/h_rcm_vs_action_client.cpp)
target_link_libraries(h_rcm_vs ${catkin_LIBRARIES})

# Executable
add_executable(crop_undistort_image_node src/crop_undistort_image_node.cpp)
target_link_libraries(crop_undistort_image_node ${catkin_LIBRARIES})

add_executable(h_rcm_vs_client_node src/h_rcm_vs_client_node.cpp)
target_link_libraries(h_rcm_vs_client_node ${catkin_LIBRARIES} h_rcm_vs)

add_executable(h_rcm_vs_server_node src/h_rcm_vs_server_node.cpp)
target_link_libraries(h_rcm_vs_server_node ${catkin_LIBRARIES})

add_executable(rcm_init_node src/rcm_init_node.cpp)
target_link_libraries(rcm_init_node ${catkin_LIBRARIES})

# Install
install(
    TARGETS h_rcm_vs
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
    DIRECTORY include/h_rcm_vs/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
    TARGETS h_rcm_vs_client_node h_rcm_vs_server_node rcm_init_node crop_undistort_image_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

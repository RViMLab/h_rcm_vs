cmake_minimum_required(VERSION 3.0.2)
project(h_rcom_vs)

# Catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_ros_planning_interface
    rcom_impl
)

# Other packages
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp moveit_ros_planning_interface rcom_impl
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
    DEPENDS EIGEN3
)

# Includes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# # Executable
# add_executable(rcom_node src/rcom_ros_node.cpp)
# target_link_libraries(rcom_node ${catkin_LIBRARIES})

# # Install
# install(
#     DIRECTORY include/rcom_impl/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(
#     TARGETS rcom_node
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

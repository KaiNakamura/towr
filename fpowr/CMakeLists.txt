cmake_minimum_required(VERSION 2.8.3)
project(fpowr)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  visualization_msgs
  message_generation
  xpp_msgs
  xpp_states
  towr
  actionlib
  actionlib_msgs
  convex_plane_decomposition_ros
)
find_package(cpptrace REQUIRED)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES FootstepPlan.action
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    ContactDatum.msg
    FootstepPlan.msg
    InitialGuess.msg
    InitialGuessArray.msg
    SingleRigidBody.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    xpp_msgs
    actionlib_msgs
    convex_plane_decomposition_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS xpp_states roscpp actionlib_msgs convex_plane_decomposition_msgs
   DEPENDS towr
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Footstep Plan Server
add_executable(footstep_plan_server src/footstep_plan_server.cc)
target_link_libraries(
  footstep_plan_server
  ${catkin_LIBRARIES}
  cpptrace::cpptrace
)
add_dependencies(
  footstep_plan_server
  ${fpowr_EXPORTED_TARGETS}
)

# Footstep Plan Client
add_executable(footstep_plan_client src/footstep_plan_client.cc)
target_link_libraries(
  footstep_plan_client
  ${catkin_LIBRARIES}
  cpptrace::cpptrace
)
add_dependencies(
  footstep_plan_client
  ${fpowr_EXPORTED_TARGETS}
)

# Planar Terrain From Bag Publisher
add_executable(planar_terrain_from_bag_publisher src/planar_terrain_from_bag_publisher.cc)
target_link_libraries(
  planar_terrain_from_bag_publisher
  ${catkin_LIBRARIES}
)
add_dependencies(
  planar_terrain_from_bag_publisher
  ${fpowr_EXPORTED_TARGETS}
)

# GridMap Republisher
add_executable(grid_map_republisher src/grid_map_republisher.cc)
target_link_libraries(
  grid_map_republisher
  ${catkin_LIBRARIES}
)
add_dependencies(
  grid_map_republisher
  ${fpowr_EXPORTED_TARGETS}
)

# PlanarRegion Republisher
add_executable(planar_region_republisher src/planar_region_republisher.cc)
target_link_libraries(
  planar_region_republisher
  ${catkin_LIBRARIES}
)
add_dependencies(
  planar_region_republisher
  ${fpowr_EXPORTED_TARGETS}
)

#############
## Install ##
#############
# Mark library for installation
# install(
#   TARGETS rviz_terrain_publisher
#           goal_pose_publisher
#           rosbag_traj_combiner
#           rosbag_geom_msg_extractor
#           ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
  DIRECTORY launch rviz bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
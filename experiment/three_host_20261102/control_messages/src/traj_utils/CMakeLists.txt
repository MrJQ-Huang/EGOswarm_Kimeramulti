cmake_minimum_required(VERSION 3.0.2)
project(traj_utils)

# Only ROS message bindings; EGO control remains on the two edge hosts.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)
add_message_files(FILES
  Bspline.msg
  DataDisp.msg
  MultiBsplines.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_msgs)

# Only ROS message bindings; EGO control remains on the two edge hosts.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs nav_msgs)
add_message_files(FILES
  AuxCommand.msg
  Corrections.msg
  Gains.msg
  LQRTrajectory.msg
  Odometry.msg
  OutputData.msg
  PPROutputData.msg
  PolynomialTrajectory.msg
  PositionCommand.msg
  SO3Command.msg
  Serial.msg
  StatusData.msg
  TRPYCommand.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs)

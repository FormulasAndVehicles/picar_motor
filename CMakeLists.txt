cmake_minimum_required(VERSION 3.0.2)
project(picar_motor)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  picar_msgs
  dynamic_reconfigure
  std_srvs
)
generate_dynamic_reconfigure_options(
  cfg/motor.cfg
)
catkin_package()
#catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  nodes/motor_node.py
  nodes/wheel_speed_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


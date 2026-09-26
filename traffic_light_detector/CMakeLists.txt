cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_detector)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs std_srvs cv_bridge)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/traffic_light_node.py scripts/wait_for_green.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

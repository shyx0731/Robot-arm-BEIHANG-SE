cmake_minimum_required(VERSION 3.0.2)
project(walle_web_console)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/web_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

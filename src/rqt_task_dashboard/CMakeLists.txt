cmake_minimum_required(VERSION 3.0.2)
project(rqt_task_dashboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  python_qt_binding
  rqt_gui
  rqt_gui_py
  cv_bridge
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rqt_task_dashboard
  scripts/web_server.py
  scripts/vision_compress_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch models web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

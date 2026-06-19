cmake_minimum_required(VERSION 3.0.2)
project(walle_gazebo_assets)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

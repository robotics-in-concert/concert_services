##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_service_gazebo)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

##############################################################################
# Installs
##############################################################################

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY services
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  PROGRAMS
    scripts/gazebo_launcher.py
    scripts/gazebo_robot_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    


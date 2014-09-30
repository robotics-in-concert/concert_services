cmake_minimum_required(VERSION 2.8.3)
project(concert_service_world_canvas)

find_package(catkin REQUIRED COMPONENTS
  world_canvas_server
)

# catkin_python_setup()

catkin_package()


install(DIRECTORY services 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

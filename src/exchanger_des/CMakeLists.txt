cmake_minimum_required(VERSION 3.10.2)
project(exchanger_des)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch meshes urdf worlds mujuco_xml config DESTINATION share/${PROJECT_NAME})

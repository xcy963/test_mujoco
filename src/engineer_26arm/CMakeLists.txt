cmake_minimum_required(VERSION 3.10.2)
project(engineer_26arm)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch meshes meshes_ee urdf  DESTINATION share/${PROJECT_NAME})

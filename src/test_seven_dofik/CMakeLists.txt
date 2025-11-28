cmake_minimum_required(VERSION 3.10)
project(test_seven_dofik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  

set(OpenCV_DIR "/home/hitcrt/libxcy/opencv-4.12.0/build")
find_package(OpenCV REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(urdf REQUIRED)
# message(WARNING "是我xcy,找到的opencv目录:${OpenCV_INCLUDE_DIRS}")

include_directories(include)


add_executable(${PROJECT_NAME}
  src/main.cpp

)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${OpenCV_LIBRARIES}
 Eigen3::Eigen

)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  std_msgs
  tf2_ros
  tf2_eigen
  geometry_msgs
  urdf
  sensor_msgs 
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch 

  DESTINATION share/${PROJECT_NAME}/
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(test_mujoco)

# 如果未设置CMAKE_BUILD_TYPE，则默认为RelWithDebInfo，以便调试
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_STANDARD 17)

# 寻找必要的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)
# find_package(lodepng REQUIRED)
find_package(glfw3  REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)   

# 查找MuJoCo库。确保它已安装在系统路径，或者通过CMAKE_PREFIX_PATH指定
# find_package(mujoco REQUIRED) 
set(MUJOCO_HOME "./mujoco-3.3.7")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_HOME}/simulate")

find_path(MUJOCO_INCLUDE_DIRS
  NAMES mujoco/mujoco.h
  PATHS ${MUJOCO_HOME}/include
  NO_DEFAULT_PATH # 只在指定路径中查找
)

find_library(MUJOCO_LIBRARIES
  NAMES mujoco
  PATHS ${MUJOCO_HOME}/lib
  NO_DEFAULT_PATH
)

if(NOT MUJOCO_INCLUDE_DIRS)
  message(FATAL_ERROR "Failed to find MuJoCo headers at ${MUJOCO_HOME}/include")
endif()
if(NOT MUJOCO_LIBRARIES)
  message(FATAL_ERROR "Failed to find libmujoco.so at ${MUJOCO_HOME}/lib")
endif()

message(STATUS "Found MuJoCo include at: ${MUJOCO_INCLUDE_DIRS}")
message(STATUS "Found MuJoCo library: ${MUJOCO_LIBRARIES}")



# # 包含头文件目录
# include_directories(
#   include
#   ${MUJOCO_INCLUDE_DIRS} # 或者直接写 /path/to/your/mujoco/include
# )
set(SIMULATE_SOURCES
  ${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc
  ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc
  ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc
  ${MUJOCO_SIMULATE_DIR}/simulate.cc
)
set(OTHER_SOURCES
  include/mujoco_ros_node.cpp
  include/mujoco_test/mujoco_simulator.cpp
)

# add_library(mujoco_sim 
#   ${SIMULATE_SOURCES}
# )
# 声明可执行文件
add_executable(mujoco_ros_node
  src/main.cpp
  ${SIMULATE_SOURCES}
  ${OTHER_SOURCES}
)
target_include_directories(mujoco_ros_node 
  PUBLIC 
  include
  mujoco-3.3.7/simulate
  ${Python3_INCLUDE_DIRS} 
  ${Python3_NumPy_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIRS}
)

# 为目标链接库
target_link_libraries(mujoco_ros_node
  ${rclcpp_LIBRARIES}
  ${MUJOCO_LIBRARIES} 
  # ${PYTHON_LIBRARIES}
  # ${Python3_INCLUDE_DIRS} 
  # ${Python3_NumPy_INCLUDE_DIRS}
  Python3::Python
  glfw
  OpenGL::GL
  lodepng
  ${OpenCV_LIBS} 
  # ${mujoco_sim}
)

# 声明ROS2相关的依赖，确保它们出现在头文件和链接路径中
ament_target_dependencies(mujoco_ros_node
  rclcpp
  std_msgs
  sensor_msgs
)

# 安装可执行文件到指定目录
install(TARGETS
  mujoco_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件和config文件
install(DIRECTORY
  launch 
  config 
  modules
  DESTINATION share/${PROJECT_NAME}/
)


# 导出依赖，使其他包能通过`ament`找到本包
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)

ament_package()
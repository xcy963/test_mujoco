add_subdirectory(serial_base)
add_compile_options(
    -Wall      #启用常见警告
    -Wextra    #启用额外警告
    -Wpedantic #严格遵循语言标准
    # -O1        # 
    -march=native  # 根据本地 CPU 架构优化
)
# include_directories(include)
# include_directories(serial_base/include) #比较丑陋
set(engineer_srcs
    engineer_serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/serial_device/serial_device.cpp  # 确保包含实现文件
)
add_library(engineer_serial engineer_serial.cpp ${engineer_srcs})
# set(Python3_ROOT_DIR "/usr")
# find_package(Python3 COMPONENTS Development NumPy)
# message(STATUS "Python3_LIBRARIES: ${Python3_LIBRARIES}")
# message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")

target_include_directories(engineer_serial PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    # ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS}
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial_base/include>  #直接在对应的cmake文件include
)#Generator expressions 😋

target_link_libraries(engineer_serial PUBLIC
  serial_base
#   ${PYTHON_LIBRARIES}

  )

# get_target_property(ENGINEER_INCLUDES engineer_serial INCLUDE_DIRECTORIES)
# message("engineer_serial include directories: ")
# message(WARNING "==========================================")
# message(WARNING "查看cmake的目录: ${ENGINEER_INCLUDES}")
# message(WARNING "==========================================")


install(TARGETS engineer_serial DESTINATION lib)
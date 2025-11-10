add_compile_options(
    -Wall      #启用常见警告
    -Wextra    #启用额外警告
    -Wpedantic #严格遵循语言标准
    # -O1        # 启用 O3 优化
    # -march=native  # 根据本地 CPU 架构优化
)
include_directories(include)

file(GLOB_RECURSE base_srcs
  "*.cpp")
add_library(serial_base SHARED ${base_srcs})
target_include_directories(serial_base PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
)#Generator expressions 😋
# message(WARNING "==========================================")
# message(WARNING "看一下他是不是把三个cpp都放进去了: ${base_srcs}")
# message(WARNING "==========================================")


install(TARGETS serial_base DESTINATION lib)
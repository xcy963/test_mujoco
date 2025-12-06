
#!/bin/bash

# ============================================
# ROS2 Colcon 编译助手脚本 (支持 Conda 环境)
# 将此脚本放在你的工作空间根目录下运行
# 使用方式：./build_helper.sh [模式]
# ============================================

# 用户配置区域：请根据你的实际项目修改以下变量
# ----------------------------------------------------
# 1. 要编译的特定包（多个包用空格分隔）
PACKAGES_SELECT="test_mujoco mujuco_rotating"
# 2. 要排除的包（多个包用空格分隔）
PACKAGES_SKIP="test_pkg large_library_pkg"
# 3. 要编译的包及其所有依赖
PACKAGE_UP_TO="important_node_pkg"
# ----------------------------------------------------

# 检查当前目录是否为 ROS2 工作空间（存在 src 目录）
if [ ! -d "src" ]; then
    echo "错误：请在 ROS2 工作空间的根目录（包含 src/ 的目录）下运行此脚本。"
    exit 1
fi

# 定义编译函数
build_packages() {
    echo "================================"
    echo "开始构建..."
    echo "工作空间：$(pwd)"
    echo "时间：$(date)"
    echo "================================"
    
    # 执行 colcon 构建命令，所有参数都会传递给该函数
    colcon build --symlink-install $@
    
    if [ $? -eq 0 ]; then
        echo -e "\n构建成功！"
        echo "你可以通过以下命令设置环境："
        echo "  source install/setup.bash"
    else
        echo -e "\n构建失败，请检查上方日志。"
    fi
}

# 处理命令行参数
case "${1}" in
    "select" | "-s")
        # 模式1：仅编译指定的包（不编译其依赖）
        echo "模式：编译特定包 [${PACKAGES_SELECT}]"
        build_packages --packages-select ${PACKAGES_SELECT}
        ;;
    "up-to" | "-u")
        # 模式2：编译指定包及其所有未构建的依赖[citation:1][citation:10]
        echo "模式：编译包及其依赖 [${PACKAGE_UP_TO}]"
        build_packages --packages-up-to ${PACKAGE_UP_TO}
        ;;
    "skip" | "-k")
        # 模式3：编译时跳过指定的包[citation:7][citation:10]
        echo "模式：排除特定包 [${PACKAGES_SKIP}]"
        build_packages --packages-skip ${PACKAGES_SKIP}
        ;;
    "all" | "-a")
        # 模式4：编译工作空间中所有包
        echo "模式：编译所有包"
        build_packages
        ;;
    "debug")
        # 模式5：调试模式（构建类型为Debug）
        echo "模式：调试模式 (Debug构建)"
        build_packages --cmake-args -DCMAKE_BUILD_TYPE=Debug
        ;;
    "release")
        # 模式6：发布模式（构建类型为Release）[citation:10]
        echo "模式：发布模式 (Release构建)"
        build_packages --cmake-args -DCMAKE_BUILD_TYPE=Release
        ;;
    "help" | "-h" )
        # 显示帮助信息
        echo "ROS2 Colcon 编译助手"
        echo "======================"
        echo "用法：./build_helper.sh [模式]"
        echo ""
        echo "可用模式："
        echo "  select, -s   : 仅编译在脚本中预设的特定包 (${PACKAGES_SELECT})"
        echo "  up-to,  -u   : 编译预设包及其所有依赖 (${PACKAGE_UP_TO})"
        echo "  skip,   -k   : 编译时跳过预设的包 (${PACKAGES_SKIP})"
        echo "  all,    -a   : 编译所有包"
        echo "  debug        : 调试模式 (Debug构建类型)"
        echo "  release      : 发布模式 (Release构建类型)[citation:10]"
        echo "  help,   -h   : 显示此帮助信息"
        echo ""
        echo "提示：要修改预设的包名，请编辑本脚本开头的变量。"
        ;;
    *)
        echo "错误：未知模式 '${1}',默认编译特定包"
        echo "模式：编译特定包 [${PACKAGES_SELECT}]"
        build_packages --packages-select ${PACKAGES_SELECT}
        exit 1
        ;;
esac


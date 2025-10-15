#!/bin/bash

#声明
echo -e "==========================================================="
echo -e "   本脚本用于在 Ubuntu 20 LTS 系统上安装 ROS2 Foxy 完整版    "
echo -e "==========================================================="

# 彩色输出函数
info() {
    echo -e "\033[1;34m[INFO] $1\033[0m"
}
success() {
    echo -e "\033[1;32m[SUCCESS] $1\033[0m"
}
warning() {
    echo -e "\033[1;33m[WARNING] $1\033[0m"
}
error() {
    echo -e "\033[1;31m[ERROR] $1\033[0m"
}

# 在脚本开头添加日志文件
LOG_FILE="ros2_foxy_install_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -i "$LOG_FILE") 2>&1

# 检查系统是否为Ubuntu 20.04（focal）
if [ "$(lsb_release -sc)" != "focal" ]; then
    error "本脚本仅支持Ubuntu 20.04 (focal)，当前系统为$(lsb_release -sc)"
fi

# 检查架构是否为amd64
if [ "$(dpkg --print-architecture)" != "amd64" ]; then
    error "本脚本仅支持amd64架构，当前架构为$(dpkg --print-architecture)"
fi

# 设置脚本退出状态
set -uo pipefail

# 记录安装失败的包
failed_packages=()

# 安装函数
install_package() {
    local pkg_name="$1"
    info "正在安装: $pkg_name"
    if ! sudo apt install -y "$pkg_name"; then
        warning "安装 $pkg_name 失败"
        failed_packages+=("$pkg_name")
        return 1
    fi
    return 0
}

# 批量安装函数
install_packages() {
    local pkgs=("$@")
    info "正在安装 ${#pkgs[@]} 个包..."
    if ! sudo apt install -y "${pkgs[@]}"; then
        warning "部分包安装失败，将单独检查"
        # 单独检查每个包
        for pkg in "${pkgs[@]}"; do
            if ! dpkg -s "$pkg" &> /dev/null; then
                warning "包 $pkg 未安装成功"
                failed_packages+=("$pkg")
            fi
        done
    fi
}

info "===== 开始安装ROS 2 Foxy ====="

# 1. 备份原有源列表
info "备份系统源列表"
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak || error "备份源列表失败"

# 2. 写入阿里云Ubuntu 20.04（focal）镜像源
info "配置阿里云镜像源"
sudo tee /etc/apt/sources.list <<-'EOF' > /dev/null
deb http://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse

deb http://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse

deb http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse

deb http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse

deb http://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse
EOF

# 3. 更新缓存并安装基本工具
info "更新系统缓存"
sudo apt update || error "更新系统缓存失败"

info "安装基本工具"
install_package "curl"

# 4. 导入ROS官方GPG密钥
info "导入ROS GPG密钥"
if ! sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; then
    warning "无法从官方源获取ROS密钥，尝试备用源"
    if ! sudo curl -sSL https://gitee.com/ohhuo/rosdistro/raw/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; then
        error "获取ROS密钥失败"
    fi
fi

# 5. 添加ROS 2 Foxy源
info "配置ROS 2源"
sudo rm -f /etc/apt/sources.list.d/ros2.list
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee -a /etc/apt/sources.list.d/ros2.list > /dev/null

# 6. 更新ROS源缓存
info "更新ROS源缓存"
sudo apt update -y --fix-missing || warning "更新ROS源缓存时出现警告"

# 7. 安装ROS 2 Foxy桌面版
info "安装ROS 2 Foxy桌面版"
install_package "ros-foxy-desktop"

# 8. 安装Python依赖
info "安装Python依赖"
python_pkgs=(
    "libpython3-dev" "python3-pip" "python3-colcon-common-extensions"
    "python3-rosdep" "python3-argcomplete"
)
install_packages "${python_pkgs[@]}"

# 9. 确保lark-parser是最新版
info "更新lark-parser"
if ! pip3 install -U lark-parser; then
    warning "更新lark-parser失败"
    failed_packages+=("lark-parser (pip)")
fi

# 10. 安装日志和XML/JSON解析依赖
info "安装日志和XML/JSON解析依赖"
log_xml_pkgs=(
    "libspdlog1" "libtinyxml2-dev" "libtinyxml2-6a"
    "libtinyxml-dev" "libyaml-cpp-dev"
)
install_packages "${log_xml_pkgs[@]}"

# 11. 安装Qt相关依赖
info "安装Qt相关依赖"
qt_pkgs=(
    "qtbase5-dev" "libqt5widgets5" "libqt5gui5" "libqt5core5a"
)
install_packages "${qt_pkgs[@]}"

# 12. 安装网络、数学和通信依赖
info "安装网络、数学和通信依赖"
net_math_pkgs=(
    "libasio-dev" "libeigen3-dev" "libcurl4-openssl-dev"
    "libconsole-bridge-dev" "libpoco-dev" "libboost-system-dev"
)
install_packages "${net_math_pkgs[@]}"

# 13. 安装3D模型加载和物理引擎
info "安装3D模型加载和物理引擎"
engine_pkgs=(
    "libassimp5" "libbullet-dev" "libogre-1.9-dev"
)
install_packages "${engine_pkgs[@]}"

# 14. 安装调试与可视化工具
info "安装调试与可视化工具"
debug_vis_pkgs=(
    "ros-foxy-rqt" "ros-foxy-rqt-common-plugins"
    "ros-foxy-rqt-graph" "ros-foxy-rqt-reconfigure"
)
install_packages "${debug_vis_pkgs[@]}"

# 15. 安装导航2（Nav2）相关包
info "安装导航2相关包"
nav_pkgs=(
    "ros-foxy-navigation2" "ros-foxy-nav2-bringup"
)
install_packages "${nav_pkgs[@]}"

# 16. 安装SLAM工具箱
info "安装SLAM工具箱"
install_package "ros-foxy-slam-toolbox"

# 17. 安装Cartographer
info "安装Cartographer"
cartographer_pkgs=(
    "ros-foxy-cartographer" "ros-foxy-cartographer-ros"
)
install_packages "${cartographer_pkgs[@]}"

# 18. 安装TurtleBot3模型与仿真环境
info "安装TurtleBot3相关包"
turtlebot_pkgs=(
    "ros-foxy-turtlebot3" "ros-foxy-turtlebot3-simulations"
)
install_packages "${turtlebot_pkgs[@]}"

# 19. 安装仿真环境支持Gazebo相关包
info "安装Gazebo相关包"
gazebo_pkgs=(
    "ros-foxy-gazebo-ros" "ros-foxy-gazebo-ros-pkgs"
    "ros-foxy-gazebo-ros-control"
)
install_packages "${gazebo_pkgs[@]}"

# 20. 安装控制系统相关包
info "安装控制系统相关包"
control_pkgs=(
    "ros-foxy-control-toolbox" "ros-foxy-ros2-control"
    "ros-foxy-ros2-controllers"
)
install_packages "${control_pkgs[@]}"

# 21. 安装机械臂运动规划
info "安装机械臂运动规划包"
moveit_pkgs=(
    "ros-foxy-moveit" "ros-foxy-moveit-setup-assistant"
)
install_packages "${moveit_pkgs[@]}"

# 22. 安装图像处理、节点组合等工具
info "安装图像处理等工具包"
image_tools_pkgs=(
    "ros-foxy-image-tools" "ros-foxy-composition"
    "ros-foxy-launch-testing" "ros-foxy-launch-testing-examples"
    "ros-foxy-diagnostics"
)
install_packages "${image_tools_pkgs[@]}"

# 23. 配置环境变量
info "配置环境变量"

if ! grep -qxF "TURTLEBOT3_MODEL=burger" ~/.bashrc; then
    echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
fi

if ! grep -qxF "LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu" ~/.bashrc; then
    echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
fi

if ! grep -qxF "source /opt/ros/foxy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
fi

# 刷新共享库缓存
sudo ldconfig || warning "刷新共享库缓存时出现警告"

# 加载环境变量
source ~/.bashrc || warning "加载环境变量时出现警告"

# 24. 配置rosdep
info "配置rosdep"
if ! command -v rosdep &> /dev/null; then
    warning "rosdep未安装，尝试安装"
    install_package "python3-rosdep"
fi

if command -v rosdep &> /dev/null; then
    # 替换rosdep源为国内源
    sudo mkdir -p /etc/ros/rosdep/sources.list.d/ || warning "创建rosdep目录失败"
    if ! sudo curl -sSL https://gitee.com/ohhuo/rosdistro/raw/master/rosdep/sources.list.d/20-default.list -o /etc/ros/rosdep/sources.list.d/20-default.list; then
        warning "下载rosdep配置失败"
    else
        sudo sed -i 's|raw.githubusercontent.com/ros/rosdistro/master|gitee.com/ohhuo/rosdistro/raw/master|g' /etc/ros/rosdep/sources.list.d/20-default.list || warning "替换rosdep源失败"
    fi

    # 初始化rosdep
    if ! sudo rosdep init; then
        warning "rosdep初始化失败"
    else
        rosdep update || warning "rosdep更新失败"
        rosdep check ros-foxy-desktop || warning "检查ros-foxy-desktop依赖失败"
    fi
else
    warning "rosdep仍未安装，无法进行配置"
    failed_packages+=("python3-rosdep")
fi

# 25. 验证安装
info "开始验证安装"

# 验证核心库
info "验证系统库"
check_library() {
    local lib_pattern="$1"
    local lib_name="$2"
    if ! ls /usr/lib/x86_64-linux-gnu/"$lib_pattern" &> /dev/null; then
        warning "未找到库: $lib_name"
        failed_packages+=("$lib_name")
    fi
}

check_library "libspdlog.so*" "libspdlog"
check_library "libQt5Widgets.so*" "libQt5Widgets"
check_library "libtinyxml2.so*" "libtinyxml2"
check_library "libtinyxml.so*" "libtinyxml"
check_library "libassimp.so*" "libassimp"
check_library "libyaml-cpp.so*" "libyaml-cpp"
check_library "libasio.so*" "libasio"
check_library "libpoco*.so" "libpoco"
check_library "libBullet*.so" "libBullet"

# 验证工具
check_command() {
    local cmd="$1"
    if ! command -v "$cmd" &> /dev/null; then
        warning "未找到命令: $cmd"
        failed_packages+=("$cmd")
    fi
}

check_command "colcon"
check_command "rosdep"

info "验证ROS 2安装"
# 强制刷新环境变量，确保ROS 2相关配置生效
info "刷新环境变量..."
if ! source ~/.bashrc; then
    warning "手动加载环境变量失败，尝试直接加载ROS 2 setup文件"
    source /opt/ros/foxy/setup.bash || warning "ROS 2 setup文件加载失败"
fi

if command -v ros2 &> /dev/null; then
    ros2 --version || warning "ros2版本检查失败"
    ros2 doctor || warning "ros2系统检查有警告"
    ros2 pkg list | wc -l || warning "查看包数量失败"
    ros2 pkg list | grep turtlebot3 || warning "未找到turtlebot3相关包"
    echo "ROS_DISTRO: $ROS_DISTRO" || warning "查看ROS_DISTRO失败"
    echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH" || warning "查看AMENT_PREFIX_PATH失败"
else
    warning "ros2命令未找到，核心安装可能失败"
    failed_packages+=("ros-foxy-desktop (核心)")
fi

info "清理APT缓存，释放磁盘空间"
sudo apt autoremove -y
sudo apt clean

# 26. 显示安装失败的包
success "===== 安装流程完成 ====="

if [ ${#failed_packages[@]} -ne 0 ]; then
    echo -e "\033[1;31m以下包安装失败，建议手动安装：\033[0m"
    for pkg in "${failed_packages[@]}"; do
        echo "- $pkg"
    done
    echo -e "\033[1;33m手动安装示例：sudo apt install -y 包名\033[0m"
else
    echo -e "\033[1;32m所有包均安装成功！\033[0m"
fi

echo -e "\033[1;33m建议重启系统以确保所有配置生效：sudo reboot\033[0m"
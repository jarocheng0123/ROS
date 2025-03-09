#!/bin/bash

#声明
echo -e "==========================================================="
echo -e "   本脚本用于在 Ubuntu 系统上安装 ROS 系统                  "
echo -e "==========================================================="

# 定义颜色常量
GREEN='\033[32m'
BLUE='\033[34m'
YELLOW='\033[33m'
CYAN='\033[36m'
ORANGE='\033[38;5;208m'
PURPLE='\033[35m'
RED='\033[31m'
NC='\033[0m'

# 声明全局变量
declare -g ROS_VERSION

# 用户交互函数
colored_read() {
    local color=$1
    local message=$2
    local var_name=$3
    read -p "$(echo -e $color$message$NC)" value
    eval "$var_name=\$value"
}

# ROS版本支持列表
declare -A ROS_VERSIONS=(
    ["galactic"]="Ubuntu 20.04 (Focal) | 2021-2022"
    ["humble"]="Ubuntu 22.04 (Jammy) | 2022-2027"
    ["iron"]="Ubuntu 22.04 (Jammy) | 2023-2024"
    ["jazzy"]="Ubuntu 24.04 (Noble) | 2024-2029"
    # 可继续添加更多版本对应关系
)

# 支持的 Ubuntu 版本列表
SUPPORTED_UBUNTU_VERSIONS=("jammy" "noble")

# 获取最新版本
latest_version=$(echo "${!ROS_VERSIONS[@]}" | tr ' ' '\n' | sort -r | head -n1)
ROS_DEFAULT_VERSION=$latest_version

# 获取当前时间、系统版本和系统处理器信息
CURRENT_TIME=$(date +"%Y%m%d%H%M%S")
SYSTEM_VERSION=$(lsb_release -cs)
SYSTEM_ARCH=$(dpkg --print-architecture)

# 日志文件初始化
LOG_FILE="install.log"
echo "==================== $(date) ====================" > $LOG_FILE

# 显示当前系统状态
echo -e "${GREEN}当前时间: $CURRENT_TIME${NC}"
echo -e "${GREEN}系统版本: $SYSTEM_VERSION${NC}"
echo -e "${GREEN}系统处理器: $SYSTEM_ARCH${NC}"

# 获取用户桌面路径
function get_desktop_path() {
    local user_home=$(eval echo ~${SUDO_USER:-$(whoami)})
    local desktop_dir="$user_home/Desktop"
    if [ -f "$user_home/.config/user-dirs.dirs" ]; then
        desktop_dir=$(grep -oP 'XDG_DESKTOP_DIR="\K[^"]+' "$user_home/.config/user-dirs.dirs")
        desktop_dir=${desktop_dir/#\$HOME/$user_home}
    fi
    echo "$desktop_dir"
}
DESKTOP_DIR=$(get_desktop_path)

# ${PURPLE}ROS安装状态检测函数，检查系统中是否已安装ROS并给出操作选项${NC}
function check_ros_installed() {
    echo -e "${BLUE}▶ 检查现有安装...${NC}"
    installed_ros=$(apt list --installed 2>/dev/null | grep -oP 'ros-\K[a-z]+(?=-ros-base)')
    if [ -n "$installed_ros" ]; then
        echo -e "${YELLOW}检测到已安装的ROS版本: ${BLUE}$installed_ros${NC}"
        PS3="请选择操作: "
        select choice in "覆盖安装" "卸载后安装" "跳过安装并继续配置"; do
            case $choice in
                "覆盖安装" | "卸载后安装")
                    uninstall_ros $installed_ros
                    if [ "$choice" = "卸载后安装" ]; then
                        echo -e "${YELLOW}卸载完成，脚本结束。${NC}"
                        exit 0
                    fi
                    return 2
                    ;;
                "跳过安装并继续配置")
                    ROS_VERSION=$installed_ros
                    return 1
                    ;;
                *) 
                    echo "无效选择，请重新输入"
                    ;;
            esac
        done
    else
        return 0
    fi
}

# ${PURPLE}卸载已安装的ROS函数${NC}
function uninstall_ros() {
    local installed_ros=$1
    echo -e "${YELLOW}正在卸载ROS...${NC}"
    sudo apt remove -y "ros-${installed_ros}-*" > /dev/null 2>&1
    sudo apt autoremove -y > /dev/null 2>&1
}

# 用户交互选择 ROS 版本
colored_read "$CYAN" "是否要自定义安装参数？(y/n) " CUSTOM_CHOICE
if [[ "$CUSTOM_CHOICE" == "y" ]]; then
    echo -e "${CYAN}\n支持的ROS版本列表："
    printf "%-8s %-20s %-15s\n" 版本 适配系统 生命周期
    for ver in "${!ROS_VERSIONS[@]}"; do
        IFS='|' read -r os cycle <<< "${ROS_VERSIONS[$ver]}"
        printf "${CYAN}%-10s ${GREEN}%-18s ${YELLOW}%-15s${NC}\n" "$ver" "$os" "$cycle"
    done

    latest_version=$(echo "${!ROS_VERSIONS[@]}" | tr ' ' '\n' | sort -r | head -n1)
    colored_read "$CYAN" "\n请输入 ROS 版本（默认最新版本 $latest_version），你可以从上面列表中选择一个版本名称输入来切换版本：" ROS_VERSION
    ROS_VERSION=${ROS_VERSION:-$latest_version}

    # 版本兼容性验证
    if [[ -n "${ROS_VERSIONS[$ROS_VERSION]}" ]]; then
        required_os=$(echo "${ROS_VERSIONS[$ROS_VERSION]}" | cut -d'(' -f2 | cut -d')' -f1 | tr '[:upper:]' '[:lower:]')
        if [[ "$SYSTEM_VERSION" != "$required_os" ]]; then
            echo -e "${RED}[错误] $ROS_VERSION 仅支持 $required_os (当前系统: $SYSTEM_VERSION)${NC}" | tee -a "$LOG_FILE"
            exit 1
        fi
    else
        echo -e "${RED}[错误] 输入的 ROS 版本 $ROS_VERSION 不支持，请参考上面的支持版本列表重新输入。${NC}" | tee -a "$LOG_FILE"
        # 增加循环让用户重新输入
        while true; do
            colored_read "$CYAN" "请重新输入有效的 ROS 版本：" ROS_VERSION
            if [[ -n "${ROS_VERSIONS[$ROS_VERSION]}" ]]; then
                required_os=$(echo "${ROS_VERSIONS[$ROS_VERSION]}" | cut -d'(' -f2 | cut -d')' -f1 | tr '[:upper:]' '[:lower:]')
                if [[ "$SYSTEM_VERSION" == "$required_os" ]]; then
                    break
                else
                    echo -e "${RED}[错误] $ROS_VERSION 仅支持 $required_os (当前系统: $SYSTEM_VERSION)${NC}" | tee -a "$LOG_FILE"
                fi
            else
                echo -e "${RED}[错误] 输入的 ROS 版本 $ROS_VERSION 不支持，请参考上面的支持版本列表重新输入。${NC}" | tee -a "$LOG_FILE"
            fi
        done
    fi
else
    # 获取最新版本
    latest_version=$(echo "${!ROS_VERSIONS[@]}" | tr ' ' '\n' | sort -r | head -n1)
    ROS_VERSION=$latest_version
fi

# 选择安装类型
colored_read "$CYAN" "选择安装类型（1 基础安装，2 全部安装，默认全部安装）：" INSTALL_TYPE

case $INSTALL_TYPE in
    1)
        INSTALL_TYPE="基础安装"
        PACKAGE_NAME="ros-$ROS_VERSION-base"
        ;;
    2)
        INSTALL_TYPE="全部安装"
        PACKAGE_NAME="ros-$ROS_VERSION-desktop-full"
        ;;
    *)
        INSTALL_TYPE="全部安装"
        PACKAGE_NAME="ros-$ROS_VERSION-desktop-full"
        ;;
esac

# 系统依赖检查（恢复依赖安装）
{
    echo -e "${BLUE}[系统] 正在检查系统依赖..."
    REQUIRED_PKGS=("curl" "gnupg2" "lsb-release")
    for pkg in "${REQUIRED_PKGS[@]}"; do
        if ! command -v "$pkg" &> /dev/null; then
            sudo apt update -y >> "$LOG_FILE" 2>&1
            sudo apt install -y "$pkg" >> "$LOG_FILE" 2>&1
        fi
    done
} | tee -a "$LOG_FILE"


# 镜像源 URL 映射
declare -A MIRROR_URLS=(
    ["阿里云"]="https://mirrors.aliyun.com/ubuntu/"
    ["清华"]="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/"
    ["华为云"]="https://mirrors.huaweicloud.com/ubuntu/"
)

# 镜像源优化函数
function optimize_mirror() {
    colored_read "$CYAN" "是否优化系统镜像源？(y/n) " MIRROR_CHOICE
    if [[ "$MIRROR_CHOICE" == "y" ]]; then
        {
            echo -e "${BLUE}[镜像] 开始测试镜像源..."
            declare -A MIRROR_SPEED
            local timeout=5
            
            # 测速逻辑
            for name in "${!MIRROR_URLS[@]}"; do
                url="${MIRROR_URLS[$name]}"
                echo -e "${CYAN}[测试] 正在测试 ${name}..."
                speed=$(timeout $timeout curl -o /dev/null -s -w "%{time_total}" "$url" || echo "999")
                MIRROR_SPEED["$name"]=$speed
            done

            # 显示结果
            echo -e "\n${CYAN}[镜像] 测试结果："
            printf "%-10s %-12s\n" 名称 响应时间
            for name in "${!MIRROR_SPEED[@]}"; do
                if (( $(echo "${MIRROR_SPEED[$name]} < 999" | bc -l) )); then
                    printf "${CYAN}%-12s ${GREEN}%-8.2fs${NC}\n" "$name" "${MIRROR_SPEED[$name]}"
                else
                    printf "${RED}%-12s 测试超时${NC}\n" "$name"
                fi
            done

            # 自动选择最优镜像
            local best_mirror=$(for name in "${!MIRROR_SPEED[@]}"; do
                echo "$name ${MIRROR_SPEED[$name]}"
            done | sort -k2n | head -1 | awk '{print $1}')
            
            # 应用镜像源
            echo -e "${GREEN}[镜像] 自动选择: $best_mirror"
            sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
            sudo sed -i "s|http://.*archive.ubuntu.com|${MIRROR_URLS[$best_mirror]}|g" /etc/apt/sources.list
            sudo apt update -qq
        } | tee -a "$LOG_FILE"
    fi
}

# ${PURPLE}ROS镜像源选择函数，测试并选择最优ROS镜像源${NC}
function select_ros_mirror() {
    echo -e "${BLUE}▶ 测试可用的ROS镜像源...${NC}"
    ROS_MIRRORS=(
        "http://packages.ros.org/ros2/ubuntu/"
        "http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/"
        "https://repo.huaweicloud.com/ros2/ubuntu/"
        "https://mirrors.ustc.edu.cn/ros2/ubuntu/"
    )
    declare -A ros_mirror_times
    for mirror in "${ROS_MIRRORS[@]}"; do
        time=$(curl -s -w "%{time_total}" -o /dev/null $mirror)
        ros_mirror_times["$mirror"]=$time
    done

    # 按响应时间排序
    IFS=$'\n' sorted_ros_mirrors=($(for mirror in "${!ros_mirror_times[@]}"; do echo "${ros_mirror_times[$mirror]} $mirror"; done | sort -n))
    unset IFS

    # 显示测试结果排名
    echo -e "${CYAN}ROS镜像源测试结果排名：${NC}"
    count=1
    for result in "${sorted_ros_mirrors[@]}"; do
        mirror=$(echo $result | cut -d' ' -f2)
        echo -e "${CYAN}$count. $mirror (响应时间: $(echo $result | cut -d' ' -f1)s)${NC}"
        ((count++))
    done

    # 默认选择最优镜像源
    default_ros_mirror=$(echo ${sorted_ros_mirrors[0]} | cut -d' ' -f2)
    read -p "$(echo -e ${CYAN}是否使用默认ROS镜像源 $default_ros_mirror？\(y/n\) 若选择 n 请输入排名编号： ${NC})" use_ros_default
    if [ "$use_ros_default" = "y" ]; then
        selected_ros_mirror=$default_ros_mirror
    elif [[ "$use_ros_default" =~ ^[0-9]+$ ]] && [ $use_ros_default -ge 1 ] && [ $use_ros_default -le ${#sorted_ros_mirrors[@]} ]; then
        index=$((use_ros_default - 1))
        selected_ros_mirror=$(echo ${sorted_ros_mirrors[$index]} | cut -d' ' -f2)
    else
        echo -e "${YELLOW}无效输入，使用默认ROS镜像源。${NC}"
        selected_ros_mirror=$default_ros_mirror
    fi

    # 备份原 ROS 源配置
    if [ -f /etc/apt/sources.list.d/ros2.list ]; then
        sudo cp /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list.bak
    fi

    # 替换为所选镜像源
    CODENAME=$(lsb_release -cs)
    ARCH=$(dpkg --print-architecture)
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null <<EOF
deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $selected_ros_mirror $CODENAME main
EOF
    sudo apt update -y >> "$LOG_FILE" 2>&1
}

# 显示进度条函数
function show_progress() {
    local total_steps=$1
    local current_step=$2
    local message=$3
    local percent=$((current_step * 100 / total_steps))
    local bar_length=50
    local filled_length=$((percent * bar_length / 100))
    local bar=$(printf "%${filled_length}s" | tr ' ' '#')
    local spaces=$(printf "%$((bar_length - filled_length))s" | tr ' ' ' ')
    echo -ne "${ORANGE}[${bar}${spaces}] ${percent}% - ${message}\r${NC}"
}

# 安装 ROS 核心组件
function install_ros_core() {
    local ros_version=$1
    local package_name=$2
    echo "传递的 ROS 版本: $ros_version"
    echo "传递的包名: $package_name"
    echo -e "${BLUE}▶ 开始安装ROS $ros_version...${NC}"

    local packages=(
        "$package_name"
        # 可以根据需要添加其他依赖包
    )
    local total_packages=${#packages[@]}
    local step=0

    for package in "${packages[@]}"; do
        ((step++))
        show_progress $total_packages $step "正在安装 $package"
        sudo apt install -y $package
    done
    echo

    # 验证安装结果
    if [ ! -d "/opt/ros/$ros_version" ]; then
        echo -e "${RED}✖ 核心组件安装失败！${NC}"
        exit 1
    fi

    # 安装额外依赖
    case $ros_version in
        "humble" | "iron" | "jazzy" | "rolling")
            sudo apt install -y python3-colcon-common-extensions python3-argcomplete python3-rosdep >> $LOG_FILE 2>&1
            ;;
        "noetic")
            sudo apt install -y python3-catkin-tools python3-rosdep >> $LOG_FILE 2>&1
            ;;
        "melodic")
            sudo apt install -y python-catkin-tools python-rosdep >> $LOG_FILE 2>&1
            ;;
    esac
}

# ${PURPLE}密钥管理函数，添加ROS安全密钥${NC}
function manage_keys() {
    echo -e "${BLUE}▶ 配置安全密钥...${NC}"
    sudo apt update -y >> $LOG_FILE 2>&1
    sudo apt install -y curl gnupg2 >> $LOG_FILE 2>&1
    KEY_URLS=(
        "https://gitee.com/ohhuo/rosdistro/raw/master/ros.asc"
        "https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc"
    )
    for url in "${KEY_URLS[@]}"; do
        if curl -sSL "$url" | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null; then
            echo -e "${GREEN}成功添加密钥: $url${NC}"
            return 0
        fi
    done
    echo -e "${RED}✖ 密钥添加失败！请检查网络连接${NC}"
    exit 1
}

# ${PURPLE}环境配置函数，设置环境变量、创建桌面快捷方式和设置开机自启动${NC}
function setup_environment() {
    local ros_version=$1
    echo "当前配置的 ROS 版本: $ros_version"
    # 设置环境变量
    read -p "$(echo -e ${CYAN}是否将 ROS 环境变量写入 .bashrc？\(y/n\) ${NC})" env_choice
    if [ "$env_choice" = "y" ]; then
        local user_home=$(eval echo ~${SUDO_USER:-$(whoami)})
        if ! grep -q "ROS_DISTRO" "$user_home/.bashrc"; then
            echo -e "\n# ROS环境配置" >> "$user_home/.bashrc"
            echo "export ROS_DISTRO=$ros_version" >> "$user_home/.bashrc"
            echo "source /opt/ros/\$ROS_DISTRO/setup.bash" >> "$user_home/.bashrc"
        fi
    fi

    # 创建桌面快捷方式
    read -p "$(echo -e ${CYAN}是否创建常用工具的快捷方式？\(y/n\) ${NC})" shortcut_choice
    if [ "$shortcut_choice" = "y" ]; then
        manage_shortcuts $ros_version
    fi
    # 开机自启动
    read -p "$(echo -e ${CYAN}是否设置 ROS 后台服务开机自启动？\(y/n\) ${NC})" autostart_choice
    if [ "$autostart_choice" = "y" ]; then
        manage_services $ros_version
    fi
}

# ${PURPLE}桌面集成函数，创建常用工具的桌面快捷方式${NC}
function manage_shortcuts() {
    local ros_version=${1:-$ROS_DEFAULT_VERSION}
    local created_count=0  # 初始化 created_count
    
    # 临时保存 ROS_VERSION 的值,这是一个bug
    local original_ros_version=$ROS_VERSION
    echo -e "临时保存值== $original_ros_version = $ROS_VERSION  =="

    source "/opt/ros/$ros_version/setup.bash"

    # 恢复 ROS_VERSION 的值,这是一个bug
    ROS_VERSION=$original_ros_version
    echo -e "恢复值==$ROS_VERSION = $original_ros_version =="

    echo -e "${BLUE}▶ 配置桌面快捷方式...${NC}"
    local USER_HOME=$(eval echo ~${SUDO_USER:-$(whoami)})
    local DESKTOP_DIR=$(get_desktop_path)
    mkdir -p "$DESKTOP_DIR"

    declare -A apps=(
        ["RViz"]="rviz2"
        ["Gazebo"]="gz sim -v4"
        ["终端"]="ros2 doctor"
        ["小乌龟"]="ros2 run turtlesim turtlesim_node"
        ["控制台"]="ros2 run demo_nodes_py listener"
        ["启动示例launch"]="ros2 launch demo_nodes_cpp talker_listener.launch.py"
        ["雷达点云显示"]="ros2 launch velodyne_description velodyne.launch.py && ros2 launch velodyne_pointcloud VLP16_points.launch.py && ros2 run rviz2 rviz2 -d $(ros2 pkg prefix velodyne_description)/share/velodyne_description/rviz/velodyne.rviz"
        ["TurtleBot3 Gazebo 仿真"]="export TURTLEBOT3_MODEL=waffle_pi && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
        ["TurtleBot3 导航启动"]="export TURTLEBOT3_MODEL=waffle_pi && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/map/map.yaml"
        ["TurtleBot3 SLAM 建图"]="export TURTLEBOT3_MODEL=waffle_pi && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true"

    )

    # 输出桌面快捷方式建立进度
    for key in "${!apps[@]}"; do
        echo -e " $key  "
    done

    for name in "${!apps[@]}"; do
        local shortcut_file="$DESKTOP_DIR/ROS_$name.desktop"
        cat > "$shortcut_file" <<EOF
[Desktop Entry]
Name=ROS2 $name
Exec=bash -c 'source $USER_HOME/.bashrc; source /opt/ros/$ros_version/setup.bash; echo ROS_DISTRO: \$ROS_DISTRO; echo LD_LIBRARY_PATH: \$LD_LIBRARY_PATH; ${apps[$name]}'
Terminal=true
Type=Application
Icon=utilities-terminal
Categories=Development;
EOF
        if [ -f "$shortcut_file" ]; then
            chmod +x "$shortcut_file"
            ((created_count++))
        fi
    done

    # 验证快捷方式
    if [ $created_count -eq ${#apps[@]} ]; then
        echo -e "${GREEN}成功创建${BLUE}${#apps[@]}${GREEN}个桌面快捷方式${NC}"
    else
        echo -e "${YELLOW}警告：部分快捷方式创建失败 (成功: $created_count/${#apps[@]})${NC}"
    fi
}

# ${PURPLE}服务管理函数，设置ROS后台服务开机自启动并立即启动服务${NC}
function manage_services() {
    local ros_version=$1
    echo -e "${BLUE}▶配置后台服务...${NC}"

    # 创建系统服务文件
    sudo tee /etc/systemd/system/ros2.service > /dev/null <<EOF
[Unit]
Description=ROS 2 Daemon
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -c "source /opt/ros/$ros_version/setup.bash && ros2 daemon start"
Restart=always
User=${SUDO_USER:-$(whoami)}

[Install]
WantedBy=multi-user.target
EOF

    # 重新加载 systemd 管理器配置
    sudo systemctl daemon-reload

    # 启用并启动服务
    sudo systemctl enable ros2.service
    sudo systemctl start ros2.service

    # 检查服务是否启动成功
    if ! sudo systemctl is-active --quiet ros2.service; then
        echo -e "${YELLOW}⚠ ROS后台服务启动失败，请检查日志：sudo journalctl -u ros2.service${NC}"
    fi
}

# ${PURPLE}安装后验证函数，检查核心命令、环境变量和服务状态${NC}
function post_install_check() {
    echo -e "\n${GREEN}=== 安装验证 ===${NC}"
    local error_flag=0

    # 核心命令检查
    declare -a required_cmds=("ros2" "rviz2" "gz")
    for cmd in "${required_cmds[@]}"; do
        if ! command -v $cmd >/dev/null; then
            echo -e "${RED}✖ 缺失组件: $cmd${NC}"
            error_flag=1
        fi
    done

    # 环境变量验证
    source "$HOME/.bashrc"
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${YELLOW}⚠ 环境变量未加载，请执行: ${BLUE}source $HOME/.bashrc${NC}"
        error_flag=1
    else
        echo -e "${GREEN}✓ 当前ROS版本: ${BLUE}$ROS_DISTRO${NC}"
        echo -e "${GREEN}✓ 安装位置: ${BLUE}/opt/ros/$ROS_DISTRO${NC}"
    fi

    # 服务状态检查
    if ! ros2 daemon status >/dev/null 2>&1; then
        echo -e "${YELLOW}⚠ ROS后台服务未运行${NC}"
        error_flag=1
    fi

    return $error_flag
}


# ${PURPLE}功能测试函数，测试ROS节点通信、RViz可视化和小乌龟仿真${NC}
function test_ros_functionality() {
    local ros_version=${1:-$ROS_DEFAULT_VERSION}
    echo "当前 ROS 版本: $ros_version "  # 添加调试信息

    if [ -f "/opt/ros/$ros_version/setup.bash" ]; then
        source "/opt/ros/$ros_version/setup.bash"
    else
        echo -e "${RED}✖ 无法找到 ROS 环境设置文件: /opt/ros/$ros_version/setup.bash${NC}"
        return 1
    fi
    
    # 检查并安装 dbus-x11
    if ! command -v dbus-launch &> /dev/null; then
        echo -e "${YELLOW}⚠ dbus-launch 未找到，正在安装 dbus-x11...${NC}"
        sudo apt-get install -y dbus-x11 >> $LOG_FILE 2>&1
    fi

    # 确保 ROS 核心服务启动
    for i in {1..3}; do
        if ros2 daemon start > /dev/null 2>&1; then
            break
        fi
        sleep 1
    done
    if ! ros2 daemon status > /dev/null 2>&1; then
        echo -e "${RED}✖ ROS 核心服务启动失败${NC}"
        return 1
    fi
    sleep 1

    echo -e "${BLUE}▶ 开始功能测试...${NC}"

    # 节点测试 - 发送和接收
    echo -e "${BLUE}  - 启动节点发送和接收测试...${NC}"
    gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; ros2 run demo_nodes_py talker"
    sleep 2
    gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; ros2 run demo_nodes_py listener"
    sleep 10
    if ros2 node list | grep -q "talker" && ros2 node list | grep -q "listener"; then
        echo -e "${GREEN}  ✓ 节点发送和接收测试成功${NC}"
    else
        echo -e "${RED}  ✖ 节点发送和接收测试失败，可能原因：节点未启动或无法正常通信。${NC}"
    fi

    # 打开 RVIZ 测试
    echo -e "${BLUE}  - 打开 RVIZ 测试...${NC}"
    gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; rviz2"
    sleep 2
    if ps -ef | grep -q "[r]viz2"; then
        echo -e "${GREEN}  ✓ RVIZ 测试成功${NC}"
    else
        echo -e "${RED}  ✖ RVIZ 测试失败，可能原因：RVIZ 启动失败或依赖缺失。${NC}"
    fi

    # 小乌龟仿真测试
    echo -e "${BLUE}  - 小乌龟仿真测试...${NC}"
    gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; ros2 run turtlesim turtlesim_node"
    sleep 2
    if ros2 node list | grep -q "turtlesim"; then
        echo -e "${GREEN}  ✓ 小乌龟仿真测试成功${NC}"
    else
        echo -e "${RED}  ✖ 小乌龟仿真测试失败，可能原因：节点未启动或无法正常通信。${NC}"
    fi
}

# ${PURPLE}最终提示函数，根据安装验证结果给出相应提示和常见问题解决建议${NC}
function final_tip() {
    echo -e "${YELLOW}常见问题解决建议：${NC}"
    echo -e "${YELLOW}- 镜像源更新失败：检查网络连接，尝试更换镜像源。${NC}"
    echo -e "${YELLOW}- 依赖缺失：手动安装缺失的依赖包。${NC}"
    echo -e "${YELLOW}=================================${NC}"
    echo -e "${YELLOW}- 重启后终端输入 ROS2 测试开机自启${NC}"

    if post_install_check; then
        local USER_HOME=$(eval echo ~$(logname))
        echo -e "\n${GREEN}✓✓✓ 安装成功！请执行以下命令：${NC}"
        echo -e "  ${YELLOW}source $USER_HOME/.bashrc${NC}"
        source $USER_HOME/.bashrc
    else
        echo -e "\n${YELLOW}⚠ 安装完成但存在警告，建议检查以上问题${NC}"
    fi
}

# 调用欢迎界面与系统检测函数
function welcome_and_check() {
    echo -e "${GREEN}=== ROS自动化安装脚本 ===${NC}"
    if [[ ! " ${SUPPORTED_UBUNTU_VERSIONS[@]} " =~ " $SYSTEM_VERSION " ]]; then
        echo -e "${RED}✖ 不支持的Ubuntu版本: $SYSTEM_VERSION，仅支持 ${SUPPORTED_UBUNTU_VERSIONS[@]}。${NC}"
        exit 1
    fi
}
welcome_and_check

# 调用系统镜像源优化函数
optimize_mirror

# 检查ROS安装状态
check_ros_installed
install_status=$?

# 密钥管理
manage_keys

# 选择ROS镜像源
select_ros_mirror

# 根据安装状态执行相应操作
case $install_status in
    0)  # 全新安装
        install_ros_core "$ROS_VERSION" "$PACKAGE_NAME"
        setup_environment "$ROS_VERSION"
        ;;
    1)  # 跳过安装
        echo -e "${YELLOW}⚠ 使用现有安装版本: ${BLUE}$ROS_VERSION${NC}"
        setup_environment "$ROS_VERSION"
        ;;
    2)  # 卸载后安装
        install_ros_core "$ROS_VERSION" "$PACKAGE_NAME"
        setup_environment "$ROS_VERSION"
        ;;
esac

# 执行安装后验证
post_install_check

# 执行功能测试
test_ros_functionality ${ROS_VERSION:-$ROS_DEFAULT_VERSION}

# 显示最终提示
final_tip
#!/bin/bash

# 获取原始用户
ORIGINAL_USER="${SUDO_USER:-$(whoami)}"

# 获取原始用户的主目录
USER_HOME=$(getent passwd "$ORIGINAL_USER" | awk -F: '{print $6}')

# 桌面路径
case "$LANG" in
    zh_CN.UTF-8|zh_CN|zh*)
        DESKTOP_PATH="${USER_HOME}/桌面"
        ;;
    *)
        DESKTOP_PATH="${USER_HOME}/Desktop"
        ;;
esac

# 驱动安装命令
sudo apt install -y ros-jazzy-ur-robot-driver ros-jazzy-ur-msgs ros-jazzy-control-msgs

# 放行 30001（RTDE）和 29999（Dashboard）端口
sudo ufw allow 29999/tcp
sudo ufw allow 30001/tcp

# 创建桌面快捷方式
cat << "EOF" > /$DESKTOP_PATH/UR5e_ROS2驱动.desktop
[Desktop Entry]
Name=UR5e_ROS2驱动
Comment=Launch UR5e ROS2 driver for URSim
Exec=bash -c 'source /opt/ros/jazzy/setup.bash && export ROBOT_IP=127.0.0.1 && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=127.0.0.1 use_fake_hardware:=false'
Icon=application-x-executable
Terminal=true
Type=Application
Categories=Development;Robot;
EOF

# 快捷方式执行权限
chmod +x /$DESKTOP_PATH/UR5e_ROS2驱动.desktop

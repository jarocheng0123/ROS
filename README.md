# [ROS 自动化安装脚本](https://fishros.org.cn/forum/topic/3923/ros%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85)

```bash
wget http://fishros.com/install -O fishros && bash fishros
```

## 一、概述
本脚本专为在 Ubuntu 系统上自动化安装 ROS（机器人操作系统）而设计，提供了用户友好的交互界面，支持多种 ROS 版本的选择，能自动检测系统环境、优化镜像源、管理密钥、安装核心组件、配置环境变量、创建桌面快捷方式、设置开机自启等功能，并在安装完成后进行验证和功能测试，最后给出相应提示和常见问题解决建议。

## 二、支持的系统和 ROS 版本
### 支持的 Ubuntu 版本
- jammy (Ubuntu 22.04)
- noble (Ubuntu 24.04)

### 支持的 ROS 版本及对应关系
| 版本  | 适配系统  | 生命周期  |
| --- | --- | --- |
| galactic | Ubuntu 20.04 (Focal) | 2021 - 2022 |
| humble | Ubuntu 22.04 (Jammy) | 2022 - 2027 |
| iron | Ubuntu 22.04 (Jammy) | 2023 - 2024 |
| jazzy | Ubuntu 24.04 (Noble) | 2024 - 2029 |

## 三、使用方法
### 1. 下载脚本
将脚本保存为一个 `.sh` 文件，例如 `ros2.sh`。

### 2. 赋予执行权限
在终端中执行以下命令：
```bash
chmod +x ros2.sh
```

### 3. 运行脚本
```bash
./ros2.sh
```

### 4. 按照提示操作
- **自定义安装参数**：询问是否要自定义安装参数，输入 `y` 可选择 ROS 版本，输入 `n` 则使用最新版本进行安装。
- **选择安装类型**：可选择基础安装或全部安装，默认全部安装。
- **优化系统镜像源**：询问是否优化系统镜像源，输入 `y` 会自动测试并选择最优镜像源。
- **选择 ROS 镜像源**：自动测试可用的 ROS 镜像源，并提供选择最优镜像源的选项。
- **环境配置**：询问是否将 ROS 环境变量写入 `.bashrc`、是否创建常用工具的快捷方式、是否设置 ROS 后台服务开机自启动。

## 四、脚本功能详细介绍
### 1. 系统信息显示
脚本会显示当前时间、系统版本和系统处理器信息，方便用户确认系统环境。

### 2. ROS 安装状态检测
检查系统中是否已安装 ROS，若已安装，提供覆盖安装、卸载后安装、跳过安装并继续配置三种操作选项。

### 3. 用户交互选择 ROS 版本
支持用户自定义选择 ROS 版本，若输入的版本不支持或与当前系统不兼容，会提示用户重新输入。

### 4. 镜像源优化
- **系统镜像源**：提供阿里云、清华、华为云等镜像源供测试，自动选择最优镜像源并更新系统。
- **ROS 镜像源**：测试多个 ROS 镜像源，根据响应时间排序，用户可选择默认最优镜像源或指定排名编号的镜像源。

### 5. 安装 ROS 核心组件
根据用户选择的 ROS 版本和安装类型，安装相应的核心组件，并在安装过程中显示进度条。安装完成后会验证安装结果，并安装额外依赖。

### 6. 密钥管理
添加 ROS 安全密钥，若添加失败会提示检查网络连接。

### 7. 环境配置
- **环境变量设置**：可选择将 ROS 环境变量写入 `.bashrc`。
- **桌面快捷方式创建**：可选择创建常用工具的桌面快捷方式，如 RViz、Gazebo 等。
- **开机自启动设置**：可选择设置 ROS 后台服务开机自启动。

### 8. 安装后验证
检查核心命令、环境变量和服务状态，若有问题会给出相应提示。

### 9. 功能测试
测试 ROS 节点通信、RViz 可视化和小乌龟仿真，若测试失败会提示可能的原因。

### 10. 最终提示
根据安装验证结果给出相应提示和常见问题解决建议，若安装成功会提示执行 `source ~/.bashrc` 命令。

## 五、日志文件
脚本会将安装过程中的信息记录到 `install.log` 文件中，方便用户查看和排查问题。

## 六、注意事项
- 运行脚本前请确保系统已联网，且具备 sudo 权限。
- 若在安装过程中遇到问题，可查看 `install.log` 文件获取详细信息，或参考最终提示中的常见问题解决建议。
- 部分功能（如桌面快捷方式创建、功能测试）可能需要图形界面支持，请确保系统具备相应环境。

## 七、安装 UR5e 驱动
### 1. 驱动安装命令
```bash
sudo apt install -y ros-jazzy-ur-robot-driver ros-jazzy-ur-msgs ros-jazzy-control-msgs
```
- **说明**：
- `ros-jazzy-ur-robot-driver` Universal Robots 机器人的官方 ROS 2 驱动程序
- `ros-jazzy-ur-msgs` Universal Robots 机器人专用的 ROS 2 消息定义 
- `ros-jazzy-control-msgs` ROS 2 机器人控制的通用消息定义

### 2.放行 30001（RTDE）和 29999（Dashboard）端口
```bash
sudo ufw allow 29999/tcp
sudo ufw allow 30001/tcp
```

### 3.创建桌面快捷方式
```bash
cat << "EOF" > ~/桌面/UR5e_ROS2驱动.desktop
[Desktop Entry]
Name=UR5e_ROS2驱动
Comment=Launch UR5e ROS2 driver for URSim
Exec=bash -c 'source /opt/ros/jazzy/setup.bash && export ROBOT_IP=127.0.0.1 && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=127.0.0.1 use_fake_hardware:=false'
Icon=application-x-executable
Terminal=true
Type=Application
Categories=Development;Robot;
EOF
```

### 4.快捷方式执行权限
```bash
chmod +x ~/桌面/UR5e_ROS2驱动.desktop
```
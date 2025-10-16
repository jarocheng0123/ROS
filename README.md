# [ROS](https://github.com/ros2/ros2/) 自动化安装脚本

## 📌 相关资源
- [动手学ROS2](https://fishros.com/d2lros2foxy/#/)
- [鱼香ROS社区](https://fishros.org.cn/forum/)
- [ROS2 中文网](http://ros2.fishros.com/)


## 一、概述
本脚本专为在 Ubuntu 系统上自动化安装 ROS（机器人操作系统）而设计，提供了用户友好的交互界面，支持多种 ROS 版本的选择，能自动检测系统环境、优化镜像源、管理密钥、安装核心组件、配置环境变量、创建桌面快捷方式、设置开机自启等功能，并在安装完成后进行验证和功能测试，最后给出相应提示和常见问题解决建议。


## 二、[Ubuntu 版本](https://ubuntu.com/about/release-cycle)与 [ROS2 版本](https://docs.ros.org/en/kilted/Releases.html)对应关系一览表

| Ubuntu 代号        | Ubuntu 版本  |   LTS   | 发布时间  | 生命周期结束 |
|--------------------|-------------|----------|----------|-------------|
| Xenial Xerus       | 16.04       | ✅      | 2016-04   | 2021-04     |
| Bionic Beaver      | 18.04       | ✅      | 2018-04   | 2023-04     |
| Focal Fossa        | 20.04       | ✅      | 2020-04   | 2025-04     |
| Jammy Jellyfish    | 22.04       | ✅      | 2022-04   | 2027-04     |
| Kinetic Kudu       | 22.10       | ❌      | 2022-10   | 2023-07     |
| Lunar Lobster      | 23.04       | ❌      | 2023-04   | 2024-01     |
| Mantic Minotaur    | 23.10       | ❌      | 2023-10   | 2024-07     |
| Noble Numbat       | 24.04       | ✅      | 2024-04   | 2029-04     |

| ROS2 代号           | 对应Ubuntu版本      | LTS  | 发布时间    |  生命周期结束   |
|--------------------|---------------------|------|------------|----------------|
| Ardent Apalone     | Ubuntu 16.04        | ❌   | 2017-12    | 2018-12        |
| Bouncy Bolson      | Ubuntu 16.04/18.04  | ❌   | 2018-07    | 2019-01        |
| Crystal Clemmys    | Ubuntu 18.04        | ❌   | 2018-12    | 2019-07        |
| Dashing Diademata  | Ubuntu 18.04        | ✅   | 2019-05    | 2021-05        |
| Eloquent Elusor    | Ubuntu 18.04/20.04  | ❌   | 2019-11    | 2020-11        |
| Foxy Fitzroy       | Ubuntu 20.04        | ✅   | 2020-06    | 2023-05        |
| Galactic Geochelone| Ubuntu 20.04        | ❌   | 2021-05    | 2022-11        |
| Humble Hawksbill   | Ubuntu 22.04        | ✅   | 2022-05    | 2027-05        |
| Iron Irwini        | Ubuntu 22.04        | ❌   | 2023-05    | 2024-11        |
| Jazzy Jalisco      | Ubuntu 24.04        | ✅   | 2024-05    | 2029-05        |


## 三、使用方法

```bash
sudo apt update && sudo apt upgrade -y # 更新系统
```

[**鱼香ROS一键安装脚本**](https://fishros.org.cn/forum/topic/3923/ros%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85/2)

```bash
wget http://fishros.com/install -O fishros && bash fishros
```

[**ROS2 一键安装脚本**](https://github.com/jarocheng0123/ROS/tree/main/sh)支持的 Ubuntu 版本

- Ubuntu 20.04 ➡️ ``ros2_foxy.sh``
- Ubuntu 22.04 ➡️ ``ros2.sh``
- Ubuntu 24.04 ➡️ ``ros2.sh``

```bash
chmod +x ros2.sh
./ros2.sh
```

### 脚本交互界面

- **自定义安装参数**：询问是否要自定义安装参数，输入 `y` 可选择 ROS 版本，输入 `n` 则使用最新版本进行安装。
- **选择安装类型**：可选择基础安装或全部安装，默认全部安装。
- **优化系统镜像源**：询问是否优化系统镜像源，输入 `y` 会自动测试并选择最优镜像源。
- **选择 ROS 镜像源**：自动测试可用的 ROS 镜像源，并提供选择最优镜像源的选项。
- **环境配置**：询问是否将 ROS 环境变量写入 `.bashrc`、是否创建常用工具的快捷方式、是否设置 ROS 后台服务开机自启动。

### 验证安装

```bash
ros2 # 查看
ros2 doctor # 检查
echo $ROS_DISTRO  # 安装版本
echo $AMENT_PREFIX_PATH # 安装路径
```

```bash
ros2 pkg list | grep "ros2cli"  # 应显示 ros2cli
ros2 pkg list | grep "demo_nodes"  # 应显示 demo_nodes_cpp/demo_nodes_py
ros2 pkg list | grep "rclcpp"  # C++客户端库
ros2 pkg list | grep "rclpy"   # Python客户端库
ros2 pkg list | grep "rqt"  # 应显示 rqt、rqt_gui、rqt_graph 等
ros2 pkg list | grep "rviz2"  # 3D可视化工具，应显示 rviz2
ros2 pkg list | grep "gazebo_ros"  # 应显示 gazebo_ros、gazebo_ros_pkgs 等
ros2 pkg list | grep "navigation2"  # 应显示 navigation2、nav2_*
ros2 pkg list | grep "slam_toolbox"  # 应显示 slam_toolbox
ros2 pkg list | grep "cartographer"  # 若安装Cartographer，应显示相关包
ros2 pkg list | grep "turtlebot3"  # 应显示 turtlebot3、turtlebot3_gazebo 等
```


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


## 七、ROS2 命令

### 7.1、基础命令验证
```bash
ros2  # 查看所有可用命令
```

<img src="png/ros2.png" width="40%">

| 子命令（Command） | 核心用途 | 关键场景 |
|-------------------|----------|----------|
| `action`          | 管理 ROS 2 Action 通信 | 长时间任务（导航、抓取）的目标发送与状态查询 |
| `bag`             | 录制/回放 ROS 2 话题数据 | 数据复现、离线调试、算法测试 |
| `component`       | 管理动态加载组件 | 模块化节点设计（多个组件共享一个进程） |
| `daemon`          | 控制 ROS 2 守护进程 | 节点发现、通信基础服务的启停与状态检查 |
| `doctor`          | 诊断 ROS 2 环境配置 | 排查环境变量、依赖、网络等问题 |
| `interface`       | 查看消息/服务/Action 类型 | 查询接口定义（如 `std_msgs/msg/String` 的字段） |
| `launch`          | 运行启动文件 | 批量启动节点、设置参数、配置重映射 |
| `lifecycle`       | 管理生命周期节点 | 规范节点状态切换（初始化→活跃→关闭） |
| `multicast`       | 测试/配置多播通信 | 多机通信时验证网络多播支持 |
| `node`            | 管理 ROS 2 节点 | 查看节点列表、信息，停止节点 |
| `param`           | 管理节点参数 | 设置/获取/删除参数（如机器人轮子半径） |
| `pkg`             | 管理 ROS 2 功能包 | 创建包、查看包路径、列已安装包 |
| `run`             | 运行单个节点（可执行文件） | 单独启动一个功能包中的节点 |
| `security`        | 配置 ROS 2 安全通信 | 加密、认证、授权（防止数据泄露/篡改） |
| `service`         | 管理 ROS 2 Service 通信 | 「请求-响应」式交互（如查询机器人位置） |
| `topic`           | 管理 ROS 2 Topic 通信 | 「发布-订阅」式交互（如传感器数据传输） |
| `wtf`             | `doctor` 的别名 | 快速诊断环境问题（口语化命令） |

### 7.2、通信机制验证（话题发布与订阅）
```bash
# 终端1：发布话题
ros2 run demo_nodes_py talker

# 终端2：订阅话题
ros2 run demo_nodes_py listener
```

### 7.3、Turtlesim仿真验证（图形界面+键盘控制）
```bash
# 终端1：启动小乌龟仿真
ros2 run turtlesim turtlesim_node

# 终端2：启动键盘控制器
ros2 run turtlesim turtle_teleop_key
```

### 7.4、Gazebo仿真环境验证 
> 国内网络可提前配置[离线模型](https://github.com/osrf/gazebo_models)，避免启动卡住。
```bash
# 启动空的Gazebo仿真环境 
ros2 launch gazebo_ros gazebo.launch.py
```

### 7.5、TurtleBot3仿真与控制验证
```bash
# 终端1：设置模型并启动仿真环境
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端2：启动键盘控制器
ros2 run turtlebot3_teleop teleop_keyboard
```

### 7.6、SLAM验证
```bash
# 终端1：启动仿真环境
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端2：启动SLAM系统
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# 终端3：键盘控制机器人移动建图
ros2 run turtlebot3_teleop teleop_keyboard

# 终端4：建图完成后，保存地图到指定目录
mkdir -p ~/turtlebot3_maps  # 创建地图保存目录
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_maps/my_map --ros-args -p use_sim_time:=True  # 保存地图到指定目录
```

### 7.7、导航与地图验证（Navigation2）
```bash
# 终端1：启动仿真环境
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端2：启动导航系统
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~/turtlebot3_maps/my_map.yaml
```

### 7.8、RViz2可视化验证
```bash
# 启动RViz2
rviz2

# 使用TurtleBot3默认配置启动RViz2
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/rviz/model.rviz
```


## 八、ROS2 调试

- [2.3 动手玩ROS2](https://fishros.com/d2lros2foxy/#/chapt2/2.4%E5%8A%A8%E6%89%8B%E7%8E%A9ROS2)
- [4.1 ROS2话题介绍](https://fishros.com/d2lros2foxy/#/chapt4/4.1ROS2%E8%AF%9D%E9%A2%98%E4%BB%8B%E7%BB%8D)
- [4.2.1 话题通信Python实现](https://fishros.com/d2lros2foxy/#/chapt4/4.2%E8%AF%9D%E9%A2%98%E9%80%9A%E4%BF%A1%E5%AE%9E%E7%8E%B0(Python))
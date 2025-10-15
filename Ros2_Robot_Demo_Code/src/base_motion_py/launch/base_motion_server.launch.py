
import os

from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法

from launch import LaunchDescription   # launch文件的描述类
from launch_ros.actions import Node    # 节点启动的描述类
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


package_dir=get_package_share_directory('base_motion_py')
remappings=[('/chassis/odom','/chassis/odom'),
            ('/chassis/cmd_vel','/chassis/cmd_vel')]

params=[
   {
      "pidType": "pidIncremental",    #PidPositional or pidIncremental
      "kp": 1.0,
      "ki": 0.0,
      "kd": 0.0,
   }
]

def generate_launch_description():     # 自动生成launch文件的函数
    
   return LaunchDescription([          # 返回launch文件的描述信息
      Node(                            # 配置一个节点的启动
         package='base_motion_py',          # 节点所在的功能包
         executable='base_motion_server',  # 节点的可执行文件名
         namespace='base_motion_py',       # 节点所在的命名空间
         name='base_motion_server',                   # 对节点重新命名
         parameters=params,           # 加载参数文件
         output="screen",
         remappings=remappings
      ),
   ])

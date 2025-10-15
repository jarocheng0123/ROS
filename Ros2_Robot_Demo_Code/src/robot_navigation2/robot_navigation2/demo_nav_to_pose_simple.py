#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import tf_transformations   # TF坐标变换库




def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    yaw=0.0
    quat=tf_transformations.quaternion_from_euler(0.0,00.0,yaw)
    goal_pose.pose.orientation.x = quat[0]
    goal_pose.pose.orientation.y = quat[1]
    goal_pose.pose.orientation.z = quat[2]
    goal_pose.pose.orientation.w = quat[3]


    navigator.goToPose(goal_pose)


if __name__ == '__main__':
    main()

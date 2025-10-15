#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import tf_transformations

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # 位置初始化
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    #添加导航点
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.5
    goal_pose1.pose.position.y = -0.55
    yaw=0
    quat=tf_transformations.quaternion_from_euler(0,0,yaw)
    goal_pose1.pose.orientation.x = quat[0]
    goal_pose1.pose.orientation.y = quat[1]
    goal_pose1.pose.orientation.z = quat[2]
    goal_pose1.pose.orientation.w = quat[3]
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.3
    goal_pose2.pose.position.y = -0.2
    yaw=0
    quat=tf_transformations.quaternion_from_euler(0,0,yaw)
    goal_pose2.pose.orientation.x = quat[0]
    goal_pose2.pose.orientation.y = quat[1]
    goal_pose2.pose.orientation.z = quat[2]
    goal_pose2.pose.orientation.w = quat[3]
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -0.8
    goal_pose3.pose.position.y = -0.3
    yaw=0
    quat=tf_transformations.quaternion_from_euler(0,0,yaw)
    goal_pose3.pose.orientation.x = quat[0]
    goal_pose3.pose.orientation.y = quat[1]
    goal_pose3.pose.orientation.z = quat[2]
    goal_pose3.pose.orientation.w = quat[3]
    goal_poses.append(goal_pose3)


    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():#检查任务是否完成
        i = i + 1
        feedback = navigator.getFeedback()#获取任务反馈
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            if now - nav_start > Duration(seconds=600.0):#超出时间取消任务
                navigator.cancelTask()

            if now - nav_start > Duration(seconds=35.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = now.to_msg()
                goal_pose4.pose.position.x = -1.6
                goal_pose4.pose.position.y = -0.3
                yaw=0
                quat=tf_transformations.quaternion_from_euler(0,0,yaw)
                goal_pose4.pose.orientation.x = quat[0]
                goal_pose4.pose.orientation.y = quat[1]
                goal_pose4.pose.orientation.z = quat[2]
                goal_pose4.pose.orientation.w = quat[3]
                goal_poses = [goal_pose4]
                nav_start = now
                navigator.followWaypoints(goal_poses)

    result = navigator.getResult()#获取最终结果
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()#关闭生命周期管理服务器

    exit(0)


if __name__ == '__main__':
    main()
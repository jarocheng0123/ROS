import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .pid_controller import *
from threading import Thread,RLock
import time
import tf_transformations
from rclpy.action import ActionServer
from rclpy import action
from hb_interface.action import BaseMotion
from hb_interface.action._base_motion import BaseMotion_Goal
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
import math

class BaseMotionServer(Node):
    def __init__(self,name):
        super().__init__(name)
        self.current_pose=[0.0,0.0]
        self.odom_sub=self.create_subscription(Odometry,"/chassis/odom",self.odomCallback,1)
        self.cmd_vel_pub=self.create_publisher(Twist,"/chassis/cmd_vel",1)
        self.declare_parameter("pidType","PidPositional")
        self.declare_parameter("kp",0.3)
        self.declare_parameter("ki",0.3)
        self.declare_parameter("kd",0.3)
        self.kp=self.get_parameter("kp").get_parameter_value().double_value
        self.ki=self.get_parameter("ki").get_parameter_value().double_value
        self.kd=self.get_parameter("kd").get_parameter_value().double_value
        pidType=self.get_parameter("pidType").get_parameter_value().string_value
        if pidType=="PidPositional":
            self.pid=PidPositional(self.kp,self.ki,self.kd,0)
        else:
            self.pid=pidIncremental(self.kp,self.ki,self.kd,0)
        self.lock=RLock()
        self.action_server=ActionServer(
            self,
            BaseMotion,
            "base_motion",
            self.executeCallback,
            goal_callback=self.goalCallback,
            handle_accepted_callback=self.acceptedCallback,
            cancel_callback=self.cancelCallback
        )

    def goalCallback(self,goal_request):
        # print("goal_request",goal_request)
        mode=[0,1,2]
        if goal_request.mode not in mode:
            return action.GoalResponse.REJECT
        if goal_request.mode==2 and goal_request.value<0:
            return action.GoalResponse.REJECT
        return action.GoalResponse.ACCEPT
    
    def acceptedCallback(self,goal_handle):
        # print("goal_handle",goal_handle)
        goal_handle.execute()

    def cancelCallback(self,cancel_request):
        # print("cancel_request",cancel_request)
        return action.CancelResponse.ACCEPT
    
    def executeCallback(self,goal_handle:ServerGoalHandle):
        speed=Twist()
        result = BaseMotion.Result()  
        feedback_msg = BaseMotion.Feedback()
        flag_1=1 if goal_handle.request.value>0 else -1
        print(f"mode:{goal_handle.request.mode},value:{goal_handle.request.value}")
        if goal_handle.request.mode==0:
            origin_pose=self.current_pose.copy()
            self.pid.setTarget(goal_handle.request.value)
            self.pid.setLimit(1.0,0.1)
            self.pid.reset()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.sucess=False
                    return result
                with self.lock:
                    dist=flag_1*((self.current_pose[0]-origin_pose[0])**2+(self.current_pose[1]-origin_pose[1])**2)**0.5
                feedback_msg.percentage=abs(dist)/abs(goal_handle.request.value)
                goal_handle.publish_feedback(feedback_msg)
                value=self.pid.calOutput(dist)
                if abs(self.pid.err)<0.05:
                    speed.linear.x=0.0
                    self.cmd_vel_pub.publish(speed)
                    break
                speed.linear.x=value
                self.cmd_vel_pub.publish(speed)
                time.sleep(0.1)
        elif goal_handle.request.mode==1:
            origin_pose=self.current_pose.copy()
            self.pid.setTarget(goal_handle.request.value)
            self.pid.setLimit(1.5,1.0)
            self.pid.reset()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.sucess=False
                    return result
                with self.lock:
                    if flag_1==1:
                        if self.current_pose[2]-origin_pose[2]<0:
                            self.current_pose[2]+=2*math.pi
                    else:
                        if self.current_pose[2]-origin_pose[2]>0:
                            self.current_pose[2]-=2*math.pi
                    yaw=(self.current_pose[2]-origin_pose[2])
                feedback_msg.percentage=abs(yaw)/abs(goal_handle.request.value)
                goal_handle.publish_feedback(feedback_msg)
                value=self.pid.calOutput(yaw)
                if abs(self.pid.err)<0.15:
                    speed.angular.z=0.0
                    self.cmd_vel_pub.publish(speed)
                    break
                speed.angular.z=value
                self.cmd_vel_pub.publish(speed)
                time.sleep(0.05)
        elif goal_handle.request.mode==2:
            for i in range(4):
                origin_pose=self.current_pose.copy()
                self.pid.setTarget(goal_handle.request.value)
                self.pid.setLimit(1.0,0.1)
                self.pid.reset()
                while rclpy.ok():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.sucess=False
                        return result
                    with self.lock:
                        dist=flag_1*((self.current_pose[0]-origin_pose[0])**2+(self.current_pose[1]-origin_pose[1])**2)**0.5
                    # print("***************",dist)
                    feedback_msg.percentage=abs(dist)/abs(goal_handle.request.value)
                    goal_handle.publish_feedback(feedback_msg)
                    value=self.pid.calOutput(dist)
                    if abs(self.pid.err)<0.05:
                        speed.linear.x=0.0
                        self.cmd_vel_pub.publish(speed)
                        break
                    speed.linear.x=value
                    self.cmd_vel_pub.publish(speed)
                    time.sleep(0.1)
                time.sleep(0.5)
                    
                origin_pose=self.current_pose.copy()
                self.pid.setTarget(math.pi/2)
                self.pid.setLimit(1.5,1.0)
                self.pid.reset()
                while rclpy.ok():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.sucess=False
                        return result
                    with self.lock:
                        if flag_1==1:
                            if self.current_pose[2]-origin_pose[2]<0:
                                self.current_pose[2]+=2*math.pi
                        else:
                            if self.current_pose[2]-origin_pose[2]>0:
                                self.current_pose[2]-=2*math.pi
                        yaw=(self.current_pose[2]-origin_pose[2])
                    feedback_msg.percentage=abs(yaw)/(math.pi/2)
                    goal_handle.publish_feedback(feedback_msg)
                    value=self.pid.calOutput(yaw)
                    if abs(self.pid.err)<0.15:
                        speed.angular.z=0.0
                        self.cmd_vel_pub.publish(speed)
                        break
                    speed.angular.z=value
                    self.cmd_vel_pub.publish(speed)
                    time.sleep(0.05)
                time.sleep(0.5)
        goal_handle.succeed() 
        result.sucess=True
        return result
        

    def odomCallback(self,msg:Odometry):
        with self.lock:
            quat=msg.pose.pose.orientation
            
            euler=tf_transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
            # print("euler",euler[2])
            self.current_pose=[msg.pose.pose.position.x,msg.pose.pose.position.y,euler[2]]
            self._ok=True


def main(args=None):
    rclpy.init(args=args)
    node=BaseMotionServer("base_motion_server")
    executor=MultiThreadedExecutor(2)
    rclpy.spin(node,executor)
    node.destroy_node()
    rclpy.shutdown()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf_transformations   # TF坐标变换库

class NAV_DEMO(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter("x",0.0)
        self.declare_parameter("y",0.0)
        self.declare_parameter("yaw",0.0)
        self.x=self.get_parameter("x").get_parameter_value().double_value
        self.y=self.get_parameter("y").get_parameter_value().double_value
        self.yaw=self.get_parameter("yaw").get_parameter_value().double_value
        self.pub=self.create_publisher(PoseStamped,"/goal_pose",1)
        self.timer=self.create_timer(0.5,self.cb)

    def cb(self):
        self.timer.cancel()
        pose=PoseStamped()
        pose.header.stamp.sec=0
        pose.header.frame_id="map"
        pose.pose.position.x=self.x
        pose.pose.position.y=self.y
        pose.pose.position.z=0.0
        quat=tf_transformations.quaternion_from_euler(0,0,self.yaw)
        pose.pose.orientation.x=quat[0]
        pose.pose.orientation.y=quat[1]
        pose.pose.orientation.z=quat[2]
        pose.pose.orientation.w=quat[3]
        self.pub.publish(pose)
        self.get_logger().info(f"导航目标点坐标:x={self.x},y={self.y},z={self.yaw}")

def main(args=None):
    rclpy.init(args=args)
    node=NAV_DEMO("nav_demo_01")
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

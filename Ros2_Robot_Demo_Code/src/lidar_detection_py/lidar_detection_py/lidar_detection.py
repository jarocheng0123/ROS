import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class LidarDetection(Node):
    def __init__(self,name):
        super().__init__(name)
        self.lidar_sub=self.create_subscription(LaserScan,"/scan",self.scanCallback,1)
        self.led_pub=self.create_publisher(Bool,"/hbsenb1/do0",1)
        self.declare_parameter("distance",0.3)
        self.distance=self.get_parameter("distance").get_parameter_value().double_value

    def scanCallback(self,msg:LaserScan):
        cnt=0
        trig=False
        for distance in msg.ranges:
            if distance<self.distance:
                trig=True
                azimuth=(msg.angle_min+cnt*msg.angle_increment)*180.0/3.14159
                self.get_logger().warn(f"Warn: Azimuth:{azimuth} Distance:{distance}")
                break
            cnt+=1
        self.setLed(trig)


    def setLed(self,state):
        msg=Bool()
        msg.data=state
        self.led_pub.publish(msg)

    

def main(args=None):
    rclpy.init(args=args)
    node=LidarDetection("lidar_detection")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
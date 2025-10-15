import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from hb_interface.action import BaseMotion
from rclpy.task import Future
from hb_interface.action._base_motion import BaseMotion_FeedbackMessage


class BaseMotionClinet(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter("mode",0)
        self.declare_parameter("value",0.3)
        self.mode=self.get_parameter("mode").get_parameter_value().integer_value
        self.value=self.get_parameter("value").get_parameter_value().double_value
        self.action_client=ActionClient(self,BaseMotion,'base_motion')

    def sendGoal(self):
        goal_msg=BaseMotion.Goal()
        goal_msg.mode=self.mode
        goal_msg.value=self.value
        self.action_client.wait_for_server()
        self.send_goal_future=self.action_client.send_goal_async(goal=goal_msg,feedback_callback=self.feefbackCallback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future:Future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("goal rejected")
            return
        self.get_logger().info("gaol accepted")
        get_result_future=goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feefbackCallback(self,feedbacl_msg:BaseMotion_FeedbackMessage):
        feedback=feedbacl_msg.feedback
        self.get_logger().info(f"Received feedback:{feedback.percentage}")

    def get_result_callback(self,future):
        result=future.result().result
        self.get_logger().info(f"Result: {result.sucess}")

def main(args=None):
    rclpy.init(args=args)
    node=BaseMotionClinet("base_motion_client")
    node.sendGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
# file: clamp_cmd_vel.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ClampCmdVel(Node):
    def __init__(self):
        super().__init__('clamp_cmd_vel')
        in_topic  = self.declare_parameter('in_topic',  '/cmd_vel').get_parameter_value().string_value
        out_topic = self.declare_parameter('out_topic', '/cmd_vel_filtered').get_parameter_value().string_value
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.cb, 10)

    def cb(self, msg: Twist):
        out = Twist()
        out.linear.x  = max(0.0, msg.linear.x)   # clamp negativo -> 0
        out.linear.y  = msg.linear.y
        out.linear.z  = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = max(0.0,msg.angular.z)
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ClampCmdVel())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

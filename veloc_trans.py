import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import yaml

def load_rob(file_name):
    with open("/home/jnkimmelman/ros2_ws/src/project_4/"+file_name) as f:
        robot = yaml.safe_load(f)
    return robot

class Vel(Node):
    def __init__(self,file_name):
        super().__init__("Velocity_Translation_Node")
        self.vel_sub = self.create_subscription(Twist,"/cmd_vel",self.vel_translate,10)
        self.velr_pub = self.create_publisher(Float64,"/vr",10)
        self.vell_pub = self.create_publisher(Float64,"/vl",10)
        self.robot_char = load_rob(file_name)
        self.velr = Float64()
        self.vell = Float64()
        self.velr.data = 0.0
        self.vell.data = 0.0

    def vel_translate(self,data=Twist()):
        self.velr.data = float(((self.robot_char['wheels']['distance']*data.angular.z)/2) + data.linear.x)
        self.vell.data = float(data.linear.x - ((self.robot_char['wheels']['distance']*data.angular.z)/2))
        self.velr_pub.publish(self.velr)
        self.vell_pub.publish(self.vell)


def main(args=None):
    rclpy.init(args=args)
    node = Vel("normal.robot")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

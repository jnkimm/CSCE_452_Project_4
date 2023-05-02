import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class Navigation(Node):
    def __init__(self):
        super().__init__("Navigation_Node")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.laser_sub = self.create_subscription(LaserScan,"/scan",self.update,10)
        
        self.pc = LaserScan()

    def update(self,data=PointCloud()):
        self.pc = data.points
        
        msg = Twist()


        #print(msg)
        self.cmd_vel_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from q import quaternion_from_euler

class Control(Node):
    def __init__(self):
        super().__init__("Robot_Control_Node")
        self.curr_r = 0.0 
        self.curr_l = 0.0
        self.vel_right = self.create_subscriber(Float64,"/vr",self.change_right,10)
        self.vel_left = self.create_subscriber(Float64,"/vl",self.change_right,10)

        self.timer = self.create_timer(0.1,self.broadcast)
        self.time

    def change_right():
        self.time
    def broadcast(self):
        
def main(args=None):
    rclpy.init(args=args)

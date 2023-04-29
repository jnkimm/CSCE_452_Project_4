import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from q import quaternion_from_euler,euler_from_quat
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import yaml
from tf2_msgs.msg import TFMessage
import math
import numpy as np
from time import time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from parse_map import parse_map
import random

def load_rob(file_name):
    with open("/home/jnkimmelman/ros2_ws/src/project_4/"+file_name) as f:
        robot = yaml.safe_load(f)
    return robot

class Laser(Node):
    def __init__(self,file_name,world):
        super().__init__("Laser_Scan_Node")

        self.tf_sub = self.create_subscription(TFMessage,"/tf",self.update_pose,10)
        self.laser_pub = self.create_publisher(PointCloud,"/scan",10)

        self.robot_char = load_rob(file_name)

        self.curr_trans = TransformStamped()

        #self.curr_scan = PointCloud()
        #self.curr_scan.header.frame_id = 'world'

        # self.get_map = self.create_subscription(OccupancyGrid,"/map",self.get_map,10)
        self.O_grid = parse_map(world)

        self.timer = self.create_timer(self.robot_char['laser']['rate'],self.broadcast)

    def update_pose(self,data=TFMessage()):
        self.curr_trans = data.transforms[0]
        print(self.curr_trans)

    # def get_map(self,data=OccupancyGrid()):
    #     self.O_grid = data

    def broadcast(self):
        curr_scan = PointCloud()
        curr_scan.header.frame_id = 'map_frame'
        i = self.robot_char['laser']['angle_min']
        increment = (self.robot_char['laser']['angle_max'] - self.robot_char['laser']['angle_min'])/self.robot_char['laser']['count']
        origin_state_rot = euler_from_quat(self.curr_trans.transform.rotation.w,
                                    self.curr_trans.transform.rotation.x,
                                    self.curr_trans.transform.rotation.y,
                                    self.curr_trans.transform.rotation.z)
        start_x = self.curr_trans.transform.translation.x + self.robot_char['body']['radius']*math.cos(origin_state_rot[2])
        start_y = self.curr_trans.transform.translation.y + self.robot_char['body']['radius']*math.sin(origin_state_rot[2])
        while i < (self.robot_char['laser']['angle_max']+increment):
            j = self.robot_char['laser']['range_min']
            if random.randint(1,10) == 1:
                point = Point32()
                point.x = float('nan')
                point.y = float('nan')
                point.z = float(0)
                curr_scan.points.append(point)
                i += increment
                continue
            while j < (self.robot_char['laser']['range_max']+0.01):
                pot_x = start_x + j*math.cos(origin_state_rot[2]+i) 
                pot_y = start_y + j*math.sin(origin_state_rot[2]+i) 
                # if (pot_x - (pot_x % self.O_grid.info.resolution)) == 0:
                #     ind_x = 0
                # else:
                ind_x = round((pot_x - (pot_x % self.O_grid.info.resolution))/self.O_grid.info.resolution)
                # if (pot_y - (pot_y % self.O_grid.info.resolution)) == 0:
                #     ind_y = 0
                # else:
                ind_y = round((pot_y - (pot_y % self.O_grid.info.resolution))/self.O_grid.info.resolution)
                print(f"({pot_x},{pot_y}),({ind_x},{ind_y}),{i},{j},{self.O_grid.info.resolution}")
                if (self.O_grid.data[ind_x+((ind_y*self.O_grid.info.width))] == 1):
                    print(f"{ind_x+ind_y*self.O_grid.info.width}")
                    err = np.random.normal(0,math.sqrt(self.robot_char['laser']['error_variance']))
                    point = Point32()
                    point.x = pot_x + err*math.cos(origin_state_rot[2]+i) 
                    point.y = pot_y + err*math.sin(origin_state_rot[2]+i) 
                    point.z = float(0)
                    curr_scan.points.append(point)
                    break
                j += 0.01
            i += increment
        self.laser_pub.publish(curr_scan)
                            




def main(args=None):
    rclpy.init(args=args)
    node = Laser("normal.robot","brick.world")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


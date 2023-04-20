import rclpy
from rclpy.node import Node
import yaml
from yaml import load,dump
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
import math
import numpy as np
from q import quaternion_from_euler

def load_map(file_name):
    with open("/home/jnkimmelman/ros2_ws/src/project_4/"+file_name) as f:
        map = yaml.safe_load(f)
    return map



class Init_Map(Node):
    def __init__(self,file_name):
        super().__init__("Map_Init_Node")

        self.file = load_map(file_name)
        self.O_grid = OccupancyGrid()
        self.O_grid.header.stamp = self.get_clock().now().to_msg()
        self.O_grid.header.frame_id = 'map_frame'
        origin = Pose()
        origin.position.x = 0.0
        origin.position.y = 0.0
        origin.position.z = 0.0
        self.O_grid.info.origin = origin
        self.O_grid.info.resolution = self.file['resolution']
        self.map_pub = self.create_publisher(OccupancyGrid,'/map',10)
        self.parse_map()

        print("about to publish")
        print(f"Initial pose x is: {self.file['initial_pose'][0]}")

        self.timer = self.create_timer(0.5,self.timer_callback)
        # self.map_pub.publish(self.O_grid)
        qos_profile = QoSProfile(depth=10)
        self.tf_broadcaster = TransformBroadcaster(self,qos=qos_profile)

        print(f"{self.file['initial_pose'][0]}")
        self.start_robot([self.file['initial_pose'][0],self.file['initial_pose'][1],self.file['initial_pose'][2]])

    def start_robot(self,transformation):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map_frame'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(transformation[0])
        t.transform.translation.y = float(transformation[1])
        t.transform.translation.z = float(0)
        quat = quaternion_from_euler(
            0,0,float(transformation[2]))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        self.map_pub.publish(self.O_grid)

    def parse_map(self):
        j = 0;
        k = 0;
        # width_found = False
        for i in self.file['map']:
            if i == "#":
                self.O_grid.data.append(1)
            elif i == ".":
                self.O_grid.data.append(0)
            elif i == "\n":
                self.O_grid.info.width = j-1
                j = 0
                k += 1
                # width_found = True
            j += 1
        print(self.O_grid.info.width)
        self.O_grid.info.height = k+1
        print(self.O_grid.info.height)

def main(args=None):
    rclpy.init(args=args)
    node = Init_Map("brick.world")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
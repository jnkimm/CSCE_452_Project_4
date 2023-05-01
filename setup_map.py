import rclpy
from rclpy.node import Node
import yaml
from yaml import load,dump
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
import math
import numpy as np

def load_map(file_name):
    with open("/home/evie/ros2_ws/src/project_4/"+file_name) as f:
        map = yaml.safe_load(f)
    return map

def parse_map(file_name):
    file = load_map(file_name)
    O_grid = OccupancyGrid()
    j = 0;
    k = 0;
    O_grid.info.resolution = file['resolution']
    # width_found = False
    for i in file['map']:
        if i == "#":
            O_grid.data.append(1)
        elif i == ".":
            O_grid.data.append(0)
        elif i == "\n":
            O_grid.info.width = j-1
            j = 0
            k += 1
            # width_found = True
        j += 1
    # print(self.O_grid.info.width)
    O_grid.info.height = k+1
    # print(self.O_grid.info.height)
    return O_grid

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class Init_Map(Node):
    def __init__(self,file_name):
        super().__init__("Map_Init_Node")

        self.file = load_map(file_name)
        self.O_grid = OccupancyGrid()
        origin = Pose()
        origin.position.x = 0.0
        origin.position.y = 0.0
        origin.position.z = 0.0
        self.O_grid.info.origin = origin
        self.O_grid.info.resolution = self.file['resolution']
        self.O_grid = parse_map(file_name)
        self.map_pub = self.create_publisher(OccupancyGrid,'/map',10)

        print("about to publish")
        print(f"Height is: {self.O_grid.info.height}, Width is: {self.O_grid.info.width}")

        self.timer = self.create_timer(0.5,self.timer_callback)
        # self.map_pub.publish(self.O_grid)
        qos_profile = QoSProfile(depth=10)
        self.tf_broadcaster = TransformBroadcaster(self,qos=qos_profile)
        self.init_tf_publisher = self.create_publisher(TFMessage(),'/tf',10)
        print(f"{self.file['initial_pose'][0]}")
        self.start_robot([self.file['initial_pose'][0],self.file['initial_pose'][1],self.file['initial_pose'][2]])

    def start_robot(self,transformation):
        t = TransformStamped()
        tf = TFMessage()
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

        # tf.transforms[0].transform.translation.x = float(transformation[0])
        # tf.transforms[0].transform.translation.y = float(transformation[1])
        # tf.transforms[0].transform.translation.z = float(0)
        # tf.transforms[0].transform.rotation.x = quat[0]
        # tf.transforms[0].transform.rotation.y= quat[1]
        # tf.transforms[0].transform.rotation.z = quat[2]
        # tf.transforms[0].transform.rotation.w = quat[3]
        tf.transforms.append(t)
        print(tf)
        self.tf_broadcaster.sendTransform(t)
        # self.init_tf_publisher.publish(tf)

    def timer_callback(self):
        self.O_grid.header.stamp = self.get_clock().now().to_msg()
        self.O_grid.header.frame_id = 'map_frame'
        self.map_pub.publish(self.O_grid)


def main(args=None):
    rclpy.init(args=args)
    node = Init_Map("brick.world")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
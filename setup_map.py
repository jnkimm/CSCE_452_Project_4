import rclpy
from rclpy.node import Node
import yaml
from yaml import load,dump
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
from q import quaternion_from_euler
from tf2_msgs.msg import TFMessage
from parse_map import parse_map,load_map



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
        self.map_pub.publish(self.O_grid)


def main(args=None):
    rclpy.init(args=args)
    node = Init_Map("brick.world")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
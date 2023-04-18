import rclpy
from rclpy.node import Node
import yaml
from yaml import load,dump
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

def load_map(file_name):
    with open(file_name) as f:
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

        self.timer = self.create_timer(0.5,self.timer_callback)
        # self.map_pub.publish(self.O_grid)

        print("pp")

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
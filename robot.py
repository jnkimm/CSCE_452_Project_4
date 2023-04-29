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

def load_rob(file_name):
    with open("/home/jnkimmelman/ros2_ws/src/project_4/"+file_name) as f:
        robot = yaml.safe_load(f)
    return robot

class Control(Node):
    def __init__(self,file_name):
        super().__init__("Robot_Control_Node")
        self.curr_r = Float64()
        self.curr_l = Float64()
        self.curr_r.data = 0.0
        self.curr_l.data = 0.0

        self.vel_right = self.create_subscription(Float64,"/vr",self.change_right,10)
        self.vel_left = self.create_subscription(Float64,"/vl",self.change_left,10)
        self.get_map = self.create_subscription(OccupancyGrid,"/map",self.get_map,10)
        self.O_grid = OccupancyGrid()

        self.tf_sub = self.create_subscription(TFMessage,"/tf",self.update_pose,10)
        self.curr_trans = TransformStamped()

        self.robot_char = load_rob(file_name)
        self.err_l = np.random.normal(1,math.sqrt(self.robot_char['wheels']['error_variance_left']))
        self.err_r = np.random.normal(1,math.sqrt(self.robot_char['wheels']['error_variance_right']))
        self.old_time_l = 0.0
        self.old_time_r = 0.0
        self.timer = self.create_timer(0.1,self.broadcast)
        self.error_timer = self.create_timer(self.robot_char['wheels']['error_update_rate'],self.error_calc)
        qos_profile = QoSProfile(depth=10)
        self.tf_broadcaster = TransformBroadcaster(self,qos=qos_profile)

        # self.declare_parameter('target_frame','base_link')
        # self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        # # self.target_frame = self.declare_parameter('base_link','robot_state_publisher')
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer,self)
    def error_calc(self):
        self.err_l = np.random.normal(1,math.sqrt(self.robot_char['wheels']['error_variance_left']))
        self.err_r = np.random.normal(1,math.sqrt(self.robot_char['wheels']['error_variance_right']))

    def get_map(self,data=OccupancyGrid()):
        self.O_grid = data

    def update_pose(self,data=TFMessage()):
        self.curr_trans = data.transforms[0]
        print(self.curr_trans)

    def change_right(self,data=Float64()):
        self.curr_r = data
        self.curr_r.data *= self.err_r
        self.old_time_r = time()
    
    def change_left(self,data=Float64()):
        self.curr_l = data
        self.curr_l.data *= self.err_l
        self.old_time_l = time()
        # print("about to call broadcast")
        # self.broadcast()
    
    def broadcast(self):
        #from_frame = 'base_link'
        #to_frame = 'world'
        # try:
        #     t_new = self.tf_buffer.lookup_transform(
        #         'base_link',
        #         'world',
        #         rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform world to base_link: {ex}')
        #     return
        # Getting robot origin cell
        if ((time() - self.old_time_l) >= 1.0):
            self.curr_l.data = 0.0
        if ((time() - self.old_time_r) >= 1.0):
            self.curr_r.data = 0.0
            
        if (self.curr_l.data != 0 or self.curr_r.data != 0):
            origin_state_rot = euler_from_quat(self.curr_trans.transform.rotation.w,
                                                self.curr_trans.transform.rotation.x,
                                                self.curr_trans.transform.rotation.y,
                                                self.curr_trans.transform.rotation.z)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map_frame'
            t.child_frame_id = 'base_link'
            # print(self.curr_trans.transform.rotation.w)

            if (self.curr_l.data == self.curr_r.data):
                d = self.curr_l.data*0.1
                t.transform.translation.x = d*math.cos(origin_state_rot[2]) + self.curr_trans.transform.translation.x
                t.transform.translation.y = d*math.sin(origin_state_rot[2]) + self.curr_trans.transform.translation.y
                t.transform.translation.z = float(0)  
                t.transform.rotation.x = self.curr_trans.transform.rotation.x
                t.transform.rotation.y = self.curr_trans.transform.rotation.y
                t.transform.rotation.z = self.curr_trans.transform.rotation.z
                t.transform.rotation.w = self.curr_trans.transform.rotation.w 
            else:           
                if ((self.curr_l.data + self.curr_r.data) == 0.0):
                    R=0
                else:  
                    R = (self.robot_char['wheels']['distance']/2)*((self.curr_r.data+self.curr_l.data)/(self.curr_r.data-self.curr_l.data))
                print(f"received left: {self.curr_l.data}\nreceived right: {self.curr_r.data}\nwheel distance: {self.robot_char['wheels']['distance']}\n")
                w = (self.curr_r.data - self.curr_l.data)/self.robot_char['wheels']['distance']
                c_x = self.curr_trans.transform.translation.x - (R*math.sin(origin_state_rot[2]))
                c_y = self.curr_trans.transform.translation.y + (R*math.cos(origin_state_rot[2]))
                print(f"w:{w}\nc_x:{c_x}\nc_y:{c_y}\n")
                print(f"curr theta {origin_state_rot} \n")
                mat_1 = np.array([[math.cos(w*0.1),-math.sin(w*0.1),0.0],
                                    [math.sin(w*0.1),math.cos(w*0.1),0.0],
                                    [0.0,0.0,1.0]])
                mat_2 = np.array([[self.curr_trans.transform.translation.x - c_x],
                                    [self.curr_trans.transform.translation.y - c_y],
                                    [origin_state_rot[2]]])
                mat_3 = np.array([[c_x],
                                    [c_y],
                                    [w*0.1]])
                new_pos = np.add(np.matmul(mat_1,mat_2),mat_3)
                print(new_pos)
                t.transform.translation.x = float(new_pos[0])
                t.transform.translation.y = float(new_pos[1])
                t.transform.translation.z = float(0)
                quat = quaternion_from_euler(
                    0,0,float(new_pos[2]))
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

            x = math.floor(t.transform.translation.x/self.O_grid.info.resolution)
            y = math.floor(t.transform.translation.y/self.O_grid.info.resolution)

            robot_cells = [[x,y]]

            # Checking the 4 faces of the circle
            # print((transformation[0]+.2))
            if (t.transform.translation.x+.2)/self.O_grid.info.resolution < (x+1):
                robot_cells.append([x+1,y])
            if (t.transform.translation.x-.2)/self.O_grid.info.resolution < (x):
                robot_cells.append([x-1,y])
            if (t.transform.translation.y+.2)/self.O_grid.info.resolution < (y+1):
                robot_cells.append([x,y+1])
            if (t.transform.translation.y-.2)/self.O_grid.info.resolution < (y):
                robot_cells.append([x,y-1])
            # Checking the 4 courners of the cell
            if (math.sqrt((x-x)**2+((y+1)-y)**2)) < .2/self.O_grid.info.resolution: #top left
                robot_cells.append([x-1,y+1])
            if (math.sqrt(((x+1)-x)**2+((y+1)-y)**2)) < .2/self.O_grid.info.resolution: #top right
                robot_cells.append([x+1,y+1])
            if (math.sqrt((x-x)**2+(y-y)**2)) < .2/self.O_grid.info.resolution: #bottom left
                robot_cells.append([x-1,y-1])
            if (math.sqrt(((x+1)-x)**2+(y-y)**2)) < .2/self.O_grid.info.resolution: #bottom right
                robot_cells.append([x+1,y-1])
            # print(robot_cells)

            for i in robot_cells:
                # print(f"in cell {i}")
                if (self.O_grid.data[i[0]+i[1]*self.O_grid.info.width] == 1):
                    return

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Control("normal.robot")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

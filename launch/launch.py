from launch import *
from launch.actions import *
from launch.event_handlers import *
from launch.events import *
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml

def load_robot(file_name):
    with open(file_name) as f:
        robot = yaml.safe_load(f)
    robot['urdf'] = disc_robot_urdf(robot)
    return robot

def disc_robot_urdf(robot):
    radius = robot['body']['radius']
    height = robot['body']['height']

    return (f"""<?xml version="1.0"?>
    <robot name="disc">
        <material name="light_blue">
            <color rgba="0.5 0.5 1 1"/>
        </material>
        <material name="dark_blue">
            <color rgba="0.1 0.1 1 1"/>
        </material>
        <material name="dark_red">
            <color rgba="1 0.1 0.1 1"/>
        </material>
        <link name="base_link">
            <visual>
                <geometry>
                    <cylinder length="{height}" radius="{radius}"/>
                </geometry>
                <material name="light_blue"/>
            </visual>
        </link>
        <link name="heading_box">
            <visual>
                <geometry>
                    <box size='{0.9*radius} {0.2*radius} {1.2*height}'/>
                </geometry>
                <material name="dark_blue"/>
            </visual>
        </link>
        <link name="laser" />
        <joint name="base_to_heading_box" type="fixed">
            <parent link="base_link"/>
            <child link="heading_box"/>
            <origin xyz='{1.45*radius} 0.0 0.0'/>
        </joint>
        <joint name="base_to_laser" type="fixed">
            <parent link="base_link"/>
            <child link="laser"/>
            <origin xyz='{1.5*radius} 0.0 0.0'/>
        </joint>
    </robot>
    """)

def generate_launch_description():

    # Arguments
    bag_arg = DeclareLaunchArgument('bag_in')
    ld = LaunchDescription([bag_arg])

    # Nodes
    robot_node = Node(
        package = "robot_state_publisher", 
        executable = "robot_state_publisher", 
        name= "robot_state_publisher", 
        output="screen", 
        parameters=[{"robot_description": load_robot("normal.robot")['urdf']}] # robot description
        )

    # starting node
    ld.add_action(robot_node)
    setup_map = ExecuteProcess(cmd = ['python3', 'project_4/setup_map.py'])
    ld.add_action(setup_map)
    
    # bag 
    bag = LaunchConfiguration('bag_in')
    ep = ExecuteProcess(cmd = ['ros2', 'bag', 'play', bag])
    ld.add_action(ep)

    return ld

import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node

def generate_launch_description() :

    # Package Directory
    package_dir = get_package_share_directory('amr_mapping')

    # URDF path
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    # Webots Launcher 
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'amr_world_with_features_occupancy_grid_mapping.wbt'))

    # Driver Node
    robot_driver = WebotsController(robot_name = 'robot', parameters = [{'robot_description' : robot_description_path},])

    # Navigator 
    navigator = Node(package = 'amr_mapping', executable = 'navigator')

    # Grid Mapper
    grid_mapper = Node(package = 'amr_mapping', executable = 'grid_mapper')


    with open(robot_description_path, 'r') as file :
        robot_description = file.read()

    # RviZ2
    rviz = Node(package = 'rviz2', executable = 'rviz2', name = 'rviz2')
   

    return LaunchDescription([webots, robot_driver, navigator, grid_mapper, rviz, launch.actions.RegisterEventHandler(event_handler = launch.event_handlers.OnProcessExit(target_action=webots, on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]))])
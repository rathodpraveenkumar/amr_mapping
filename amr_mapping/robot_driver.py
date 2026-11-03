"THIS NODE IS RESPONSIBLE FOR MAKING INSTANCES OF THE WEBOTS WORLDS DEVICES AND MANAGES THE ROBOTS MOTORS"

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
from geometry_msgs.msg import PoseStamped


class driveMyRobot :

    def init(self, webots_node, properties) :
        print("DEBUG: DRIVER INIT STARTED", flush=True)
        self.__robot = webots_node.robot

        # Robot Node
        robot_node = self.__robot.getFromDef("Robot")
        
        # Motors
        self.__rotational_motor_L = self.__robot.getDevice("rotational_motor_L")
        self.__rotational_motor_R = self.__robot.getDevice("rotational_motor_R")

        # Positional Sensors 
        self.__position_sensor_L = self.__robot.getDevice("position_sensor_L")
        self.__position_sensor_R = self.__robot.getDevice("position_sensor_R")

        # Lidar 
        self.__lidar = self.__robot.getDevice("lidar")

        # Translation field 
        self.__translation_field = robot_node.getField("translation")

        # Rotational field
        self.__rotational_field = robot_node.getField("rotation")

        # Webots timestep
        time_step = int(self.__robot.getBasicTimeStep())

        # Initializing the instances

        # Left Motor
        self.__rotational_motor_L.setVelocity(0.)
        self.__rotational_motor_L.setPosition(float('inf'))

        # Right Motor        
        self.__rotational_motor_R.setVelocity(0.)
        self.__rotational_motor_R.setPosition(float('inf'))

        # Left Position Sensor
        self.__position_sensor_L.enable(time_step)

        # Right Position Sensor
        self.__position_sensor_R.enable(time_step)

        # Lidar
        self.__lidar.enable(time_step)
        self.__lidar.enablePointCloud()

        # Check if Node is made
        if not rclpy.ok() :
            rclpy.init(args = None)

        # Node with name motor_driver 
        self.__node = rclpy.create_node("motor_driver")


        # Variables

        # Robots wheel's radius 
        self.__wheel_radius = 0.05
        
        # Distance between wheels
        self.__half_distance_between_wheels = 0.125

        # Twist variable 
        self.__target_twist = Twist()



        # Subscribers

        # cmd_vel Subscriber
        self.__node.create_subscription(Twist, "cmd_vel", self.__callback_cmd_vel, 10)        



        # Publishers

        # Lidar scan publisher
        self.__lidarscan_publisher = self.__node.create_publisher(LaserScan, "scan", 10)
        
        # Positional sensor publishers
        self.__position_sensor_left_publisher = self.__node.create_publisher(Float64, "position_sensor_left", 10)

        self.__position_sensor_right_publisher = self.__node.create_publisher(Float64, "position_sensor_right", 10)

        # Ground Truth Publisher    
        self.__ground_truth_publisher = self.__node.create_publisher(PoseStamped, "ground_truth", 10)

        print("DEBUG: DRIVER INIT COMPLETED", flush=True)

    # Call back methods

    # cmd_vel Call back 
    def __callback_cmd_vel(self, twist) : 
        self.__target_twist = twist


    def step(self) :
        print("DEBUG: DRIVER STEP EXECUTING", flush=True)
        rclpy.spin_once(self.__node, timeout_sec = 0)

        # Ground Truth 
        ground_truth = self.__translation_field.getSFVec3f()

        # Orirentation
        rotation = self.__rotational_field.getSFRotation()

        ground_truth_msg = PoseStamped()
        ground_truth_msg.pose.position.x = ground_truth[0]
        ground_truth_msg.pose.position.y = ground_truth[1]
        ground_truth_msg.pose.position.z = ground_truth[2]
        ground_truth_msg.pose.orientation.x = 0.
        ground_truth_msg.pose.orientation.y = 0.
        ground_truth_msg.pose.orientation.z = np.sin(rotation[3]/2)
        ground_truth_msg.pose.orientation.w = np.cos(rotation[3]/2)
        ground_truth_msg.header.stamp = self.__node.get_clock().now().to_msg()
        self.__ground_truth_publisher.publish(ground_truth_msg)

        # positon sensors data and Publishing
        position_sensor_left_value = self.__position_sensor_L.getValue()
        position_sensor_right_value = self.__position_sensor_R.getValue()

        ps_left_msg = Float64()
        ps_right_msg = Float64()

        ps_left_msg.data = position_sensor_left_value
        ps_right_msg.data = position_sensor_right_value

        self.__position_sensor_left_publisher.publish(ps_left_msg)
        self.__position_sensor_right_publisher.publish(ps_right_msg)

        # Lidar Data and Publishing
        lidar_scan = self.__lidar.getRangeImage()
        lidar_scan = [float(scan) if np.isfinite(float(scan)) else 0. for scan in lidar_scan]

        lidar_scan_msg = LaserScan()
        lidar_scan_msg.header.stamp = self.__node.get_clock().now().to_msg()
        lidar_scan_msg.header.frame_id = "laser_link"
        
        lidar_scan_msg.angle_min = float(-np.pi)
        lidar_scan_msg.angle_max = float(np.pi)
        lidar_scan_msg.angle_increment = float(-np.pi/180)
        lidar_scan_msg.range_max = float(15)
        lidar_scan_msg.range_min = float(0.01)
        lidar_scan_msg.ranges = lidar_scan

        self.__lidarscan_publisher.publish(lidar_scan_msg)


        # Command velocity
        linear_velocity = self.__target_twist.linear.x
        angular_velocity = self.__target_twist.angular.z
        
        command_velocity_left = (linear_velocity - angular_velocity*self.__half_distance_between_wheels)/self.__wheel_radius
        command_velocity_right = (linear_velocity + angular_velocity*self.__half_distance_between_wheels)/self.__wheel_radius

        self.__rotational_motor_L.setVelocity(command_velocity_left)
        self.__rotational_motor_R.setVelocity(command_velocity_right)

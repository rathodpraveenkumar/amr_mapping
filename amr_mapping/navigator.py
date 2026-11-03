"This node is Responsible for navigation "

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class myRobotNavigator(Node) :
    def __init__(self) :
        super().__init__("my_robot_navigator")

        # cmd_vel Publisher
        self.__cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # Timer
        self.create_timer(0.01, self.__navigator)


    def __navigator(self) :
        "PUBLISHES THE cmd_vel"
        # cmd_vel Variable
        cmd_vel = Twist()

        # Parameter setting
        cmd_vel.linear.x = 0.1
        cmd_vel.angular.z = 0.0

        # Publish
        self.__cmd_vel_publisher.publish(cmd_vel)

def main(args=None) :
    rclpy.init(args = args)
    navigator = myRobotNavigator()
    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()

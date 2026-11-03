"This node is Responsible for mapping "

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class gridMapper(Node) :
    def __init__(self, map_length, map_width, cell_length, cell_width, lo, l_free, l_occ) :
        super().__init__("grid_mapper")

        # Subscribers

        # LaserScan Subscriber
        self.create_subscription(LaserScan, "scan", self.__callback_laser_scan, 10)
        self.__laser_scan_msg = LaserScan()

        # GroundTruth Subscriber
        self.create_subscription(PoseStamped, "ground_truth", self.__callback_ground_truth)
        self.__ground_truth_msg = PoseStamped()

        # Publishers

        # Map Publisher
        self.__map_publisher = self.create_publisher(OccupancyGrid, "map", 10)
        self.map_msg = OccupancyGrid()


        # Timer     
        self.timer = self.create_timer(0.5, self.__publish_map)

        # Mapper implementation

        # Map and Cell dimensions
        self.__map_length = map_length
        self.__map_width = map_width

        self.__cell_length = cell_length
        self.__cell_width = cell_width

        # Rows and Columns
        if not (self.get_row_and_col()) :
            self.__N = 0
            self.__M = 0

        else :
            self.__N, self.__M = self.get_row_and_col()

        # Log Likelihoods
        self.lo = lo
        self.l_free = l_free
        self.l_occ = l_occ

        # Set Initial Likelihood
        self.likelihood = self.set_initial_likelihood()



    # CallBack functions

    # LaserScan Callback
    def __callback_laser_scan(self, laser_scan_msg) :
        self.__laser_scan_msg = laser_scan_msg
        self.__new_laser_scan = True

    # GroundTruth Callback
    def __callback_ground_truth(self, ground_truth_msg) :
        self.__ground_truth_msg = ground_truth_msg
        self.__new_ground_truth = True

    def __get_pose(self) :
        "Returns Pose from Ground truth"

    def __get_measurement(self) :
        "Returns Measurement from LaserScan and maximum distance and Angle Resolution"

    # Map Publisher
    def __publish_map(self) :
        "PUBLISHES MAP IF BOTH POSE AND MEASUREMENT ARE AVALIABLE"
        if self.__new_ground_truth and self.__new_laser_scan :

            x_t = self.__get_pose()
            z_t, z_max, ang_res = self.__get_measurement()

            self.likelihood = self.binary_occupancy_grid_mapping(x_t = x_t, z_t = z_t, z_max = z_max, alpha = self.__cell_length, beta = ang_res)


            
            map_msg =  self.__create_map_message()
    
        self.__map_publisher.publish(map_msg)
        self.reset_variables()



    def __create_map_message(self) :
        self.map_msg = OccupancyGrid()

        self.map_msg.header.frame_id = 'world'
        self.map_msg.header.stamp = self.get_clock().now().to_msg()

        self.map_msg.info.resolution = 0.4
        self.map_msg.info.width = self.__M
        self.map_msg.info.height = self.__N

        self.map_msg.info.origin.position.x = -self.__cell_width
        self.map_msg.info.origin.position.y = -self.__cell_length
        self.map_msg.info.origin.position.z = 0.0

        self.map_msg.info.origin.orientation.x = 0.0
        self.map_msg.info.origin.orientation.y = 0.0
        self.map_msg.info.origin.orientation.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0

        self.map_msg.data = self.get_map_data_from_likelihood()


    def get_map_data_from_likelihood(self) :
        "Returns the log likelihood in compatible form to ROS OccupancyGrid instance"
        return round(100*(1/(1 + np.exp(-self.likelihood))))


    def reset_variables(self) :
        self.__new_ground_truth = False
        self.__new_laser_scan = False


    def get_row_and_col(self) :
        "Returns the number of Rows and Columns of the map"

        N = 0
        M = 0
        # Check if the parameters are valid
        if np.min((self.__map_length, self.__map_width, self.__cell_length, self.__cell_width)) <= 0 :
            return 0

        # Calculate the number of Rows and Columns
        else :
            N = self.__map_width/self.__cell_width
            M = self.__map_length/self.__cell_length

        return int(N), int(M)

    def get_row(self) : 
        return self.__N

    def get_col(self) :
        return self.__M

    def set_initial_likelihood(self) :
        "Sets 1/2 as the Initial likelihood to each cell"

        self.likelihood = np.array([[self.lo for j in range(self.__M)] for i in range(self.__N)])
        

    def get_matrix_indices(self, i) :
        """ Returns the row number and column number of the matrix of size NxM"
        n  {0, 1, 2, 3, 4, ............, N - 1}
        m  {0, 1, 2, 3, 4, ............, M - 1}
            """
    
        m = i%self.__M
        n = (i - m)/self.__M

        return int(n), int(m) 

    def get_cell_center(self, i) :
        "Returns the center points pf ith cell"
        n, m = self.get_matrix_indices(i)

        x = -self.map_width/2 + (m + 0.5)*self.cell_width
        y = -self.map_length/2 + (n + 0.5)*self.cell_length

        return x, y

    def is_in_perceptual_range(self, i, x_t, ranges, angle_min, angle_increment, z_max,beta):

        # Get cell centre in world coordinates
        m_x, m_y = self.get_cell_center(i)

        # Robot pose
        x_R, y_R, theta_R = x_t

        # Assumption: LiDAR is at robot origin
        # and aligned with robot heading

        # Relative position of cell
        dx = m_x - x_R
        dy = m_y - y_R

        # Distance from LiDAR to cell
        r = np.sqrt(dx**2 + dy**2)

        # Bearing of cell relative to LiDAR
        phi = self.wrap_angle(
            np.arctan2(dy, dx) - theta_R
        )

        # Find nearest beam
        K = len(ranges)

        k = int(round(
            (phi - angle_min) / angle_increment
        ))

        # Handle full-circle LiDAR
        if np.isclose(
            K * abs(angle_increment),
            2 * np.pi,
            atol=1.5 * abs(angle_increment)
        ):
            k %= K

        # Reject invalid beam indices
        elif not (0 <= k < K):
            return False, None, r, phi

        # 7. Calculate angular difference
        theta_k = angle_min + k * angle_increment

        angular_error = abs(
            self.wrap_angle(phi - theta_k)
        )

        # 8. Check beam angular width and sensing range
        if angular_error <= beta / 2 and r <= z_max:

            return True, k, r, phi

        return False, None, r, phi


    def wrap_angle(self, angle) :
        return (angle + np.pi)%(2*np.pi) - np.pi


    def inverse_sensor_model(self, r, phi, z_t, z_max, alpha) :
        "Returns the likelihood of the map cell"

    
        r = r
        phi = phi

        if r > min(z_max, z_t + alpha/2) :
            return self.lo

        elif (z_t < z_max) and np.abs(z_t - r) <= alpha/2 :
            return self.l_occ

        elif r < z_t :
            return self.l_free

        return self.lo


    def binary_occupancy_grid_mapping(self, x_t, z_t, z_max, alpha, beta) :
        """Returns the approximated likelihood of map m given the state and the measurement at time t"
            It has been assumed the Initial position or the state of the Robot is known

            m - map of size NxM
            N - number of Rows in the map
            M - number of columns in the map
            x_t - State(pose) of the robot at time t
            z_t - measurement at time t
            likelihood (NxM)- holds the likelihood of each map Cell 

            """

        # Iterate over map cells
        for i in range(self.rows()*self.cols()) :

            # Get the row and column of the ith map cell
            n, m = self.get_matrix_indices(i = i)        

            # Likelihood of the cell
            lkd = self.likelihood[n, m]
            lkd_initial = lkd
            # Cell
            cell = self.get_cell_center(i)

            angle_min = np.radians(180)

            angle_increment = -np.radians(1)
        
            # Check if the map cell is in the perceptual range of the robot or specifically the sensor
            inside, k, r, phi = self.is_in_perceptual_range(i = i, x_t = x_t, ranges = z_t, angle_min = angle_min, angle_increment = angle_increment, z_max = z_max, beta = beta)

            

            if inside :
                measured_range = z_t[k]
                #Update the likelihood of the cell   
                lkd = lkd + self.inverse_sensor_model(r = r, phi = phi, z_t = measured_range, z_max = z_max, alpha = alpha) - self.lo

            else :
                lkd = lkd

            self.likelihood[n, m] = lkd



        

def main(args = None) :
    rclpy.init(args = args)
    grid_mapper = gridMapper(20., 20., 0.40, 0.40, 0., -0.8473, 0.8473)
    rclpy.spin_once(grid_mapper)

    grid_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == 'main' :
    main()

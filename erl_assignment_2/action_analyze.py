import rclpy
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class AnalyzeAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('analyze_marker', 0.1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.img_sub = self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        self.publisher_proc_img = self.create_publisher(Image, '/robot/processed_image', 10)

        self.marker_sub = self.create_subscription(Int32, '/aruco/marker_id', self.marker_id_callback, 10)
        
        # Current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.distance_initial_status = 0.0

        self.arrived = 0

        self.alligning = False

        self.scan_data = None

        # --- PID VARIABLES ---
        self.prev_error_x = 0.0     # Derivative term
        self.integral_error_x = 0.0 # Integral term

        # PID gains
        self.kp = 0.001   # Proportional
        self.ki = 0.000005 # Integral
        self.kd = 0.00005   # Derivative
        # ---------------------

        # waypoints map
        self.waypoints = {
            "wp1": [-6.0, -6.0],
            "wp2": [-6.0, 6.0],
            "wp3": [6.0, -6.0],
            "wp4": [6.0, 6.0],
            "wp_start": [0.0, 0.0]
        }

        self.bridge = CvBridge()
        self.latest_image = None

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Analyze Mission Management
        self.collected_ids = []      # List of received IDs
        self.sorted_ids = []         # Sorted list to visit
        self.current_target_idx = 0  # Index of the current marker in the sorted list
        self.is_list_sorted = False  # Sorting flag
        self.navigating_to_target = False # Navigation state flag
        
        # ID -> Waypoint Map (To be populated dynamically)
        self.id_to_waypoint_map = {}

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_callback(self, msg):
        self.scan_data = msg

    def image_callback(self, msg):
        """
        Callback executed every time a frame arrives from the camera.
        It ONLY converts the image and saves it in self.latest_image.
        """
        try:
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def marker_id_callback(self, msg):
        """
        Callback that receives the marker ID just found from the Rotate action.
        Saves the ID and associates the current robot position (approximated to the nearest waypoint).
        """
        marker_id = msg.data
        if marker_id not in self.collected_ids:
            self.collected_ids.append(marker_id)
            
            # Find the nearest waypoint to the current position to associate it with the ID
            closest_wp = self.get_closest_waypoint()
            self.id_to_waypoint_map[marker_id] = closest_wp
            
            self.get_logger().info(f"New ID collected: {marker_id} associated with {closest_wp}")

    def get_closest_waypoint(self):
        """Find the name of the nearest waypoint to the current position."""
        min_dist = float('inf')
        closest = None
        for name, coords in self.waypoints.items():
            dist = math.sqrt((coords[0] - self.current_x)**2 + (coords[1] - self.current_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest = name
        return closest

    def get_repulsive_force(self):
        """
        Calculate a repulsive vector based on nearby obstacles.
        Returns (force_x, force_y) in the robot's frame.
        """
        if self.scan_data is None:
            return 0.0, 0.0

        rep_x = 0.0
        rep_y = 0.0
        
        # APF Parameters
        detection_distance = 1.0 # Consider obstacles only within 1 meter
        gain = 0.002 # How strong the repulsion is

        angle = self.scan_data.angle_min
        for r in self.scan_data.ranges:
            if r < detection_distance and r > 0.1: # Ignore infinite or too small values
                # The force is inversely proportional to the distance
                force = gain / (r * r)
                
                rep_x -= force * math.cos(angle)
                rep_y -= force * math.sin(angle)
            
            angle += self.scan_data.angle_increment

        return rep_x, rep_y

    def stop_robot(self):
        """Helper function to stop the robot immediately"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def navigate_to_coords(self, target_x, target_y):
        """
        Helper function to move the robot towards X,Y coordinates.
        Returns True if arrived, False otherwise.
        """
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)
        
        if self.distance_initial_status == 0.0:
            self.distance_initial_status = dist_error

        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.current_yaw
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        attr_x = 1.5 * math.cos(yaw_error)
        attr_y = 1.5 * math.sin(yaw_error)
        rep_x, rep_y = self.get_repulsive_force()
        total_x = attr_x + rep_x
        total_y = attr_y + rep_y
        final_yaw_error = math.atan2(total_y, total_x)

        msg = Twist()
        
        if dist_error < 0.2: # Arrival tolerance
            self.cmd_pub.publish(Twist()) # Stop
            return True 
        
        elif abs(final_yaw_error) > 0.2:
            msg.angular.z = 0.5 * final_yaw_error
            self.cmd_pub.publish(msg)
        else:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            
        return False

    def do_work(self):

        # Sorting (done only once)
        if not self.is_list_sorted:
            self.sorted_ids = sorted(self.collected_ids)
            self.get_logger().info(f"All markers found! Visit order: {self.sorted_ids}")
            self.is_list_sorted = True
            self.current_target_idx = 0
            #self.navigating_to_target = True

        # Visit loop
        if self.current_target_idx < len(self.sorted_ids):
            target_id = self.sorted_ids[self.current_target_idx]
            
            # Retrieve where this ID is located
            target_wp_name = self.id_to_waypoint_map.get(target_id)
            if not target_wp_name:
                self.get_logger().error(f"Lost location for ID {target_id}!")
                self.current_target_idx += 1 # Skip
                return

            target_coords = self.waypoints[target_wp_name]

            self.get_logger().info(f"Visiting Marker {target_id} at {target_wp_name}...", throttle_duration_sec=2)
            
            # Navigate to the waypoint associated with the marker
            if self.arrived != 1:
                self.arrived = self.navigate_to_coords(target_coords[0], target_coords[1])
            
            if self.arrived:
                self.get_logger().info(f"Arrived at Marker {target_id}.")
                
                # Convert latest image to grayscale
                img_gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)

                # Detect markers
                corners, ids, _ = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)
                self.get_logger().info(f"Analyzed Marker {target_id} successfully.")
                
                if ids is not None and len(ids) > 0 and target_id in ids:
                    self.get_logger().info(f"ids recognized: {ids}")

                    target_corners = None

                    if ids is not None:
                        for i, id in enumerate(ids):
                            if id == target_id:
                                target_corners = corners[i][0]
                                break

                    # target_corners[i][j] where i is the chosen corner and j is the chosen axis
                    marker_center_x = (target_corners[0][0] + target_corners[1][0] + target_corners[2][0] + target_corners[3][0]) / 4
                    marker_center_y = (target_corners[0][1] + target_corners[1][1] + target_corners[2][1] + target_corners[3][1]) / 4

                    # Image parameters
                    image_center_x = self.latest_image.shape[1] / 2

                    error_x = image_center_x - marker_center_x
                    self.get_logger().info(f"error_x: {error_x}.")

                    # computing marker_perceived_width as difference between corner 0 (top left corner) and corner 1 (top right corner) then doing norm
                    marker_perceived_width = np.linalg.norm(target_corners[0] - target_corners[1])
                    self.get_logger().info(f"marker_perceived_width: {marker_perceived_width}.")

                    is_alligned = False
                    is_close = False

                    # error_x is computed as an average in pixels so it will have values multiples of 0.25
                    if abs(error_x) < 10:
                        is_alligned = True

                    # marker_perceived_width is in pixels, so we set a threshold in pixels too (1 TO NOT GET ANY CLOSER)
                    if abs(marker_perceived_width) > 1:
                        is_close = True

                    msg = Twist()

                    if is_alligned == False:
                    
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.linear.z = 0.0
                        msg.angular.x = 0.0
                        msg.angular.y = 0.0

                        # --- PID CALCULATION ---
                        dt = 0.1 # Action cycle time

                        # Proportional
                        P = self.kp * error_x

                        # Integral
                        self.integral_error_x += error_x * dt
                        # Clamp
                        self.integral_error_x = max(min(self.integral_error_x, 10000), -10000) 
                        I = self.ki * self.integral_error_x

                        # Derivative
                        D = self.kd * (error_x - self.prev_error_x) / dt
                        self.prev_error_x = error_x # Update for next cycle

                        # PID Output
                        angular_z_output = P + I + D
                        # -------------------

                        msg.angular.z = angular_z_output

                        self.alligning = True

                        self.cmd_pub.publish(msg)

                        return

                    elif is_alligned == True and is_close == False:
                    
                        msg.linear.x = 0.4
                        msg.linear.y = 0.0
                        msg.linear.z = 0.0
                        msg.angular.x = 0.0
                        msg.angular.y = 0.0

                        # --- PID CALCULATION ---
                        dt = 0.1 # Action cycle time

                        # Proportional
                        P = self.kp * error_x

                        # Integral
                        self.integral_error_x += error_x * dt
                        # Clamp
                        self.integral_error_x = max(min(self.integral_error_x, 10000), -10000) 
                        I = self.ki * self.integral_error_x

                        # Derivative
                        D = self.kd * (error_x - self.prev_error_x) / dt
                        self.prev_error_x = error_x # Update for next cycle

                        # PID Output
                        angular_z_output = P + I + D
                        # -------------------

                        msg.angular.z = angular_z_output

                        self.cmd_pub.publish(msg)

                        return

                    else:
                    
                        self.stop_robot()

                        # draw circle around the marker in the image
                        marker_center = (int(marker_center_x), int(marker_center_y))
                        cv2.circle(self.latest_image, marker_center, 45, (0, 255, 0), 3)

                        # Publish processed image
                        try:
                            proc_img_msg = self.bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
                            self.publisher_proc_img.publish(proc_img_msg)
                        except Exception as e:
                            self.get_logger().error(f'Error publishing processed image: {e}')

                        cv2.namedWindow("Marker Found", cv2.WINDOW_NORMAL)
                        cv2.resizeWindow("Marker Found", 600, 600)
                        self.get_logger().info("Opening window to show marker found...", throttle_duration_sec=5)
                        cv2.imshow("Marker Found", self.latest_image)
                        cv2.waitKey(3000)

                        self.alligning = False

                        # Reset for next target
                        self.distance_initial_status = 0.0 
                        self.current_target_idx += 1 # next target

                        # Provide feedback
                        self.send_feedback(float(self.current_target_idx)/len(self.sorted_ids), f"Visited {target_id}")
                        self.arrived = 0

                        # Reset PID errors for the new target
                        self.prev_error_x = 0.0
                        self.integral_error_x = 0.0
                else:
                    # --- NO MARKER FOUND ---
                    if not self.alligning:
                        msg = Twist()
                        msg.angular.z = 0.3
                        self.cmd_pub.publish(msg)

        else:
            # All markers visited
            # wait a bit before closing any windows
            cv2.waitKey(3000)
            # close any OpenCV windows
            cv2.destroyAllWindows()

            self.finish(True, 1.0, "Analysis Completed")

def main(args=None):
    rclpy.init(args=args)
    node = AnalyzeAction()
    node.set_parameters([rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'analyze_marker')])
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
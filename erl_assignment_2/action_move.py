import rclpy
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan

class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('move', 0.1) # 'move' must match the name in PDDL
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.distance_initial_status = 0.0

        self.scan_data = None
        
        # Waypoints map
        self.waypoints = {
            "wp1": [-6.0, -6.0],
            "wp2": [-6.0, 6.0],
            "wp3": [6.0, -6.0],
            "wp4": [6.0, 6.0],
            "wp_start": [0.0, 0.0]
        }

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_callback(self, msg):
        self.scan_data = msg

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
            if r < detection_distance and r > 0.1:
                # The force is inversely proportional to the distance
                force = gain / (r * r)

                rep_x -= force * math.cos(angle)
                rep_y -= force * math.sin(angle)
            
            angle += self.scan_data.angle_increment

        return rep_x, rep_y

    def do_work(self):
        # Read arguments from PDDL: (?r ?from ?to) -> args[2] is the destination
        args = self.current_arguments
        destination_id = args[2] 
        
        if destination_id not in self.waypoints:
            self.get_logger().error(f"Waypoint {destination_id} not found!")
            self.finish(False, 0.0, "Waypoint unknown")
            return

        target_x, target_y = self.waypoints[destination_id]
        
        # Calculate errors
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)

        if self.distance_initial_status == 0.0:
            self.distance_initial_status = dist_error

        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.current_yaw
        
        # Normalize angle
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        attr_x = 1.5 * math.cos(yaw_error)
        attr_y = 1.5 * math.sin(yaw_error)

        # Calculate Repulsive Vector (Obstacles)
        rep_x, rep_y = self.get_repulsive_force()

        # Total Vector
        total_x = attr_x + rep_x
        total_y = attr_y + rep_y

        # Calculate the new driving direction based on the total vector
        final_yaw_error = math.atan2(total_y, total_x)

        msg = Twist()

        # progress status computation and send feedback
        progress = 1.0 - (dist_error / self.distance_initial_status)
        if progress < 0.0:
            progress = 0.0
        self.send_feedback(progress, f"Rotating to {destination_id}")

        # Movement logic (P-Controller)
        if dist_error < 0.2: # Tolerance reached
            self.cmd_pub.publish(Twist()) # Stop
            self.distance_initial_status = 0.0
            self.finish(True, 1.0, "Arrived") # SUCCESS
        elif abs(final_yaw_error) > 0.2:
            # Rotate towards the target
            msg.angular.z = 0.5 * final_yaw_error
            self.cmd_pub.publish(msg)
        else:
            # Go straight
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveAction()
    node.set_parameters([rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move')])
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
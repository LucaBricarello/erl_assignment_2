import rclpy
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion

class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('move', 0.1) # 'move' deve matchare il nome nel PDDL
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Posizione corrente
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Mappa dei waypoint (Hardcoded per semplicità, o caricabile da params)
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

    def do_work(self):
        # 1. Leggi gli argomenti dal PDDL: (?r ?from ?to) -> args[2] è la destinazione
        args = self.current_arguments
        destination_id = args[2] 
        
        if destination_id not in self.waypoints:
            self.get_logger().error(f"Waypoint {destination_id} not found!")
            self.finish(False, 0.0, "Waypoint unknown")
            return

        target_x, target_y = self.waypoints[destination_id]
        
        # 2. Calcola errori
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.current_yaw
        
        # Normalizza angolo
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        msg = Twist()

        # 3. Logica di movimento (semplice P-Controller)
        if dist_error < 0.2: # Tolleranza raggiunta
            self.cmd_pub.publish(Twist()) # Stop
            self.finish(True, 1.0, "Arrived") # SUCCESSO
        elif abs(yaw_error) > 0.2:
            # Ruota verso il target
            msg.angular.z = 0.5 * yaw_error
            self.cmd_pub.publish(msg)
            # Feedback di progresso (opzionale)
            self.send_feedback(0.0, f"Rotating to {destination_id}")
        else:
            # Vai dritto
            msg.linear.x = 0.5
            msg.angular.z = 0.0 # Piccola correzione se vuoi: 0.2 * yaw_error
            self.cmd_pub.publish(msg)
            self.send_feedback(dist_error, f"Moving to {destination_id}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveAction()
    node.set_parameters([rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move')])
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
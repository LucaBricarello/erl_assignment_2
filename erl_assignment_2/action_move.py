import rclpy
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan

class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('move', 0.1) # 'move' deve matchare il nome nel PDDL
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Posizione corrente
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.distance_initial_status = 0.0

        self.scan_data = None
        
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

    def scan_callback(self, msg):
        self.scan_data = msg

    def get_repulsive_force(self):
        """
        Calcola un vettore repulsivo basato sugli ostacoli vicini.
        Ritorna (force_x, force_y) nel frame del robot.
        """
        if self.scan_data is None:
            return 0.0, 0.0

        rep_x = 0.0
        rep_y = 0.0
        
        # Parametri APF
        detection_distance = 1.0 # Considera ostacoli solo entro 1 metro
        gain = 0.002 # Quanto forte è la repulsione

        angle = self.scan_data.angle_min
        for r in self.scan_data.ranges:
            if r < detection_distance and r > 0.1: # Ignora valori infiniti o troppo piccoli
                # La forza è inversamente proporzionale alla distanza
                force = gain / (r * r)
                
                # Il vettore ostacolo punta VERSO l'ostacolo.
                # Noi vogliamo andare nel verso OPPOSTO, quindi usiamo -cos e -sin.
                # Nota: stiamo sommando vettori nel frame locale del robot
                rep_x -= force * math.cos(angle)
                rep_y -= force * math.sin(angle)
            
            angle += self.scan_data.angle_increment

        return rep_x, rep_y

    def do_work(self):
        # Leggi gli argomenti dal PDDL: (?r ?from ?to) -> args[2] è la destinazione
        args = self.current_arguments
        destination_id = args[2] 
        
        if destination_id not in self.waypoints:
            self.get_logger().error(f"Waypoint {destination_id} not found!")
            self.finish(False, 0.0, "Waypoint unknown")
            return

        target_x, target_y = self.waypoints[destination_id]
        
        # Calcola errori
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)

        if self.distance_initial_status == 0.0:
            self.distance_initial_status = dist_error

        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.current_yaw
        
        # Normalizza angolo
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        attr_x = 1.5 * math.cos(yaw_error)
        attr_y = 1.5 * math.sin(yaw_error)

        # Calcola Vettore Repulsivo (Ostacoli)
        rep_x, rep_y = self.get_repulsive_force()

        # Vettore Totale
        total_x = attr_x + rep_x
        total_y = attr_y + rep_y

        # Calcola la nuova direzione di guida basata sul vettore totale
        final_yaw_error = math.atan2(total_y, total_x)

        msg = Twist()

        # progress status computation and send feedback
        progress = 1.0 - (dist_error / self.distance_initial_status)
        if progress < 0.0:
            progress = 0.0
        self.send_feedback(progress, f"Rotating to {destination_id}")

        # 3. Logica di movimento (semplice P-Controller)
        if dist_error < 0.2: # Tolleranza raggiunta
            self.cmd_pub.publish(Twist()) # Stop
            self.distance_initial_status = 0.0
            self.finish(True, 1.0, "Arrived") # SUCCESSO
        elif abs(final_yaw_error) > 0.2:
            # Ruota verso il target
            msg.angular.z = 0.5 * final_yaw_error
            self.cmd_pub.publish(msg)
        else:
            # Vai dritto
            msg.linear.x = 0.5
            msg.angular.z = 0.0 # Piccola correzione se vuoi: 0.2 * final_yaw_error
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
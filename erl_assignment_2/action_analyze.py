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
        
        # Posizione corrente
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.distance_initial_status = 0.0

        self.arrived = 0

        self.alligning = False

        self.scan_data = None

        # --- VARIABILI PID ---
        self.prev_error_x = 0.0     # Per il termine Derivativo
        self.integral_error_x = 0.0 # Per il termine Integrale
        
        # Guadagni PID (Tarabili)
        self.kp = 0.001   # Proporzionale (era 0.001, alzato leggermente)
        self.ki = 0.000005 # Integrale (molto basso, corregge errore a regime)
        self.kd = 0.00005   # Derivativo (smorza le oscillazioni)
        # ---------------------

        # Mappa dei waypoint (Hardcoded per semplicità, o caricabile da params)
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
        self.aruco_params = cv2.aruco.DetectorParameters_create() # O DetectorParameters_create() su vecchie versioni opencv

        # Gestione Missione Analyze
        self.collected_ids = []      # Lista degli ID ricevuti
        self.sorted_ids = []         # Lista ordinata da visitare
        self.current_target_idx = 0  # Indice del marker corrente nella lista ordinata
        self.is_list_sorted = False  # Flag per ordinamento
        self.navigating_to_target = False # Flag stato navigazione
        
        # Mappa ID -> Waypoint (Da popolare dinamicamente o staticamente se noto)
        # Nota: Se il robot trova ID=12 mentre è al WP1, dovresti salvare questa associazione.
        # Per ora, assumiamo che tu voglia solo visitare gli ID. Ma DOVE sono?
        # Se non salviamo la posizione QUANDO li troviamo, non sappiamo dove tornare.
        # SOLUZIONE PROVVISORIA: Supponiamo che gli ID siano associati ai WP in ordine di scoperta
        # o che tu abbia una logica per dedurre la posizione.
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
        Callback che riceve l'ID del marker appena trovato dall'azione Rotate.
        Salva l'ID e associa la posizione corrente del robot (approssimata al waypoint più vicino).
        """
        marker_id = msg.data
        if marker_id not in self.collected_ids:
            self.collected_ids.append(marker_id)
            
            # Trova il waypoint più vicino alla posizione attuale per associarlo all'ID
            closest_wp = self.get_closest_waypoint()
            self.id_to_waypoint_map[marker_id] = closest_wp
            
            self.get_logger().info(f"New ID collected: {marker_id} associated with {closest_wp}")

    def get_closest_waypoint(self):
        """Trova il nome del waypoint più vicino alla posizione attuale."""
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
        Funzione helper per muovere il robot verso coordinate X,Y.
        Ritorna True se arrivato, False altrimenti.
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
        
        if dist_error < 0.2: # Tolleranza arrivo
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
        
        #if len(self.collected_ids) < 4: 
        #    self.get_logger().info(f"Waiting for markers... Found {len(self.collected_ids)}/4", throttle_duration_sec=2)
        #    return 

        # Ordinamento (fatto una sola volta)
        if not self.is_list_sorted:
            self.sorted_ids = sorted(self.collected_ids)
            self.get_logger().info(f"All markers found! Visit order: {self.sorted_ids}")
            self.is_list_sorted = True
            self.current_target_idx = 0
            #self.navigating_to_target = True

        # Ciclo di visita
        if self.current_target_idx < len(self.sorted_ids):
            target_id = self.sorted_ids[self.current_target_idx]
            
            # Recupera dove si trova questo ID
            target_wp_name = self.id_to_waypoint_map.get(target_id)
            if not target_wp_name:
                self.get_logger().error(f"Lost location for ID {target_id}!")
                self.current_target_idx += 1 # Skip
                return

            target_coords = self.waypoints[target_wp_name]

            self.get_logger().info(f"Visiting Marker {target_id} at {target_wp_name}...", throttle_duration_sec=2)
            
            # Naviga verso il waypoint associato al marker
            if self.arrived != 1:
                self.arrived = self.navigate_to_coords(target_coords[0], target_coords[1])
            
            if self.arrived:
                self.get_logger().info(f"Arrived at Marker {target_id}.")
                # Qui in futuro aggiungeremo la logica "analizza, cerchia, scatta foto"
                
                # Convert latest image to grayscale
                img_gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)

                # Detect markers
                corners, ids, _ = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)
                
                if ids is not None and len(ids) > 0 and target_id in ids:
                    self.get_logger().info(f"Analyzed Marker {target_id} successfully.")

                    #QUA LOGICA DI ALLINEAMENTO E DISEGNO CERCHIO E PUBBLICAZIONE IMG

                    target_corners = None

                    if ids is not None:
                        for i, id in enumerate(ids):
                            if id == target_id:
                                target_corners = corners[i][0]
                                break

                    # target_corners[i][j] where i is the choosen corner and j is the choosen axis
                    marker_center_x = (target_corners[0][0] + target_corners[1][0] + target_corners[2][0] + target_corners[3][0]) / 4
                    marker_center_y = (target_corners[0][1] + target_corners[1][1] + target_corners[2][1] + target_corners[3][1]) / 4

                    # Image parameters
                    image_center_x = self.latest_image.shape[1] / 2

                    error_x = image_center_x - marker_center_x
                    self.get_logger().info(f"error_x: {error_x}.")

                    # computing marker_perceived_width as difference between corner 0 (high left corner) and corner 1 (high right corner) then doing norm
                    marker_perceived_width = np.linalg.norm(target_corners[0] - target_corners[1])
                    self.get_logger().info(f"marker_perceived_width: {marker_perceived_width}.")

                    is_alligned = False
                    is_close = False

                    # error_x is computed as an average in pixels so it will have values multiples of 0.25
                    if abs(error_x) < 8:
                        is_alligned = True

                    # marker_perceived_width is in pixels, so we set a threshold in pixels too (1 TO NOT GET ANY CLOSER)
                    if abs(marker_perceived_width) > 1:
                        is_close = True

                    #k_p = 0.001

                    msg = Twist()

                    if is_alligned == False:
                    
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.linear.z = 0.0
                        msg.angular.x = 0.0
                        msg.angular.y = 0.0
                        #msg.angular.z = k_p * error_x

                        # --- CALCOLO PID ---
                        dt = 0.1 # Tempo ciclo dell'azione (definito in __init__)

                        # Proporzionale
                        P = self.kp * error_x

                        # Integrale (Accumulo errore)
                        self.integral_error_x += error_x * dt
                        # Clamp per evitare windup eccessivo (opzionale ma consigliato)
                        self.integral_error_x = max(min(self.integral_error_x, 10000), -10000) 
                        I = self.ki * self.integral_error_x

                        # Derivativo (Variazione errore)
                        D = self.kd * (error_x - self.prev_error_x) / dt
                        self.prev_error_x = error_x # Aggiorno per il prossimo ciclo

                        # Output PID
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
                        #msg.angular.z = k_p * error_x

                        # --- CALCOLO PID ---
                        dt = 0.1 # Tempo ciclo dell'azione (definito in __init__)

                        # Proporzionale
                        P = self.kp * error_x

                        # Integrale (Accumulo errore)
                        self.integral_error_x += error_x * dt
                        # Clamp per evitare windup eccessivo (opzionale ma consigliato)
                        self.integral_error_x = max(min(self.integral_error_x, 10000), -10000) 
                        I = self.ki * self.integral_error_x

                        # Derivativo (Variazione errore)
                        D = self.kd * (error_x - self.prev_error_x) / dt
                        self.prev_error_x = error_x # Aggiorno per il prossimo ciclo

                        # Output PID
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

                        # Resetta per il prossimo target
                        self.distance_initial_status = 0.0 
                        self.current_target_idx += 1 # Passa al prossimo

                        # Feedback di progresso
                        self.send_feedback(float(self.current_target_idx)/len(self.sorted_ids), f"Visited {target_id}")
                        self.arrived = 0

                        # Resetta errori PID per il nuovo target
                        self.prev_error_x = 0.0
                        self.integral_error_x = 0.0
                else:
                    # --- NESSUN MARKER ---
                    if not self.alligning:
                        msg = Twist()
                        msg.angular.z = 0.3
                        self.cmd_pub.publish(msg)

        else:
            # Tutti i marker visitati
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
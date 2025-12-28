import rclpy
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int32

class RotateAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('rotate_and_detect', 0.1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_sub = self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        # Publisher per l'ID del marker trovato (Richiesto da te)
        self.marker_pub = self.create_publisher(Int32, '/aruco/marker_id', 10)

        self.bridge = CvBridge()
        self.latest_image = None
        
        # Lista per tenere traccia dei marker già trovati
        self.found_markers = []

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create() # O DetectorParameters_create() su vecchie versioni opencv

    def image_callback(self, msg):
        """
        Callback executed every time a frame arrives from the camera.
        It ONLY converts the image and saves it in self.latest_image.
        """
        try:
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def do_work(self):
        if self.latest_image is None:
            return # Aspetta immagine

        self.get_logger().info(f"SEARCH phase", throttle_duration_sec=2)

        # Convert latest image to grayscale
        img_gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

        msg = Twist()

        # Variabile per tracciare se abbiamo trovato un NUOVO marker in questo frame
        new_marker_found = False
        found_id = -1

        if ids is not None and len(ids) > 0:
            # Controllo tutti gli ID visibili nel frame (potrebbero essercene più di uno)
            for i in range(len(ids)):
                current_id = int(ids[i][0])
                
                # <--- CONTROLLO: Se NON è nella lista dei già trovati
                if current_id not in self.found_markers:
                    found_id = current_id
                    new_marker_found = True
                    break # Ne abbiamo trovato uno nuovo, usciamo dal ciclo for

        # Logica di decisione
        if new_marker_found:
            # --- MARKER NUOVO TROVATO ---
            
            # Aggiungo alla lista per non rilevarlo più in futuro
            self.found_markers.append(found_id) # <--- AGGIUNTO
            
            # Stop robot
            self.cmd_pub.publish(Twist()) 
            
            # Pubblico ID
            id_msg = Int32()
            id_msg.data = found_id
            self.marker_pub.publish(id_msg)
            
            self.get_logger().info(f"Marker found: {found_id}. Added to list {self.found_markers}.")
            self.finish(True, 1.0, "Marker Detected")
            
        else:
            # --- NESSUN MARKER O SOLO MARKER GIÀ VISTI ---
            # Se vedo un marker vecchio, devo comunque continuare a ruotare per cercarne uno nuovo
            
            if ids is not None and len(ids) > 0:
                 self.get_logger().info(f"Only old markers detected {ids.flatten()}. Keep searching...", throttle_duration_sec=2)

            msg.angular.z = 0.3
            self.cmd_pub.publish(msg)
            self.send_feedback(0.5, "Searching...")

def main(args=None):
    rclpy.init(args=args)
    node = RotateAction()
    node.set_parameters([rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'rotate_and_detect')])
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
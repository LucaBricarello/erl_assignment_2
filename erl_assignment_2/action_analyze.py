import rclpy
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class AnalyzeAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('analyze_marker', 0.1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_sub = self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        self.proc_img_pub = self.create_publisher(Image, '/robot/processed_image', 10)
        
        self.bridge = CvBridge()
        self.latest_image = None

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def do_work(self):
        if self.latest_image is None:
            return

        # Detection per centramento
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(self.latest_image, aruco_dict, parameters=parameters)

        if ids is None:
            # Se perdo il marker durante l'analisi, fallisco o ruoto piano
            # Per ora ruoto piano per ritrovarlo
            msg = Twist()
            msg.angular.z = 0.2
            self.cmd_pub.publish(msg)
            return

        # Prendo il primo marker trovato (si suppone sia quello rilevato dalla rotazione)
        c = corners[0][0]
        marker_center_x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4
        marker_width = np.linalg.norm(c[0] - c[1])
        
        image_center_x = self.latest_image.shape[1] / 2
        error_x = image_center_x - marker_center_x
        
        # Logica di controllo (dal tuo codice originale)
        msg = Twist()
        k_p = 0.005 # Tuning
        
        is_aligned = abs(error_x) < 5
        is_close = marker_width > 90 # Pixel threshold
        
        if is_aligned and is_close:
            # STOP e Processamento
            self.cmd_pub.publish(Twist())
            
            # Disegna cerchio
            center_int = (int(marker_center_x), int((c[0][1] + c[2][1])/2))
            cv2.circle(self.latest_image, center_int, 45, (0, 255, 0), 3)
            
            # Pubblica immagine processata
            out_msg = self.bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
            self.proc_img_pub.publish(out_msg)
            
            self.get_logger().info("Analysis Complete!")
            self.finish(True, 1.0, "Analyzed")
            
        elif not is_aligned:
            # Ruota per centrare
            msg.angular.z = k_p * error_x
            self.cmd_pub.publish(msg)
        else:
            # Avvicinati
            msg.linear.x = 0.2
            msg.angular.z = k_p * error_x # Mantieni allineamento
            self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AnalyzeAction()
    node.set_parameters([rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'analyze_marker')])
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
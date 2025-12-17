import rclpy
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class RotateAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('rotate_and_detect', 0.1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_sub = self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.marker_found = False

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            pass

    def do_work(self):
        if self.latest_image is None:
            return # Aspetta immagine

        # 1. Detection ArUco
        img_gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict, parameters=parameters)

        msg = Twist()

        # 2. Se trovo un marker
        if ids is not None and len(ids) > 0:
            # Stop immediato
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"Marker found: {ids[0]}")
            self.finish(True, 1.0, "Marker Detected")
        else:
            # 3. Ruota per cercare
            msg.angular.z = 0.4
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
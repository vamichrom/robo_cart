import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
import obj_tracker.process_image as proc

class DetectObj(Node):

    def __init__(self):
        super().__init__('detect_obj')

        self.get_logger().info('Looking for the object...')
        self.image_sub = self.create_subscription(Image,"/image_in",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.obj_pub  = self.create_publisher(Point,"/detected_obj",1)

        self.declare_parameter('tuning_mode', False)

        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",0)
        self.declare_parameter("y_max",100)
        self.declare_parameter("h_min",0)
        self.declare_parameter("h_max",180)
        self.declare_parameter("s_min",0)
        self.declare_parameter("s_max",255)
        self.declare_parameter("v_min",0)
        self.declare_parameter("v_max",255)
        self.declare_parameter("sz_min",0)
        self.declare_parameter("sz_max",100)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }

        self.bridge = CvBridge()

        if(self.tuning_mode):
            proc.create_tuning_window(self.tuning_params)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            if (self.tuning_mode):
                self.tuning_params = proc.get_tuning_params()

            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            # Keep the biggest point
            # They are already converted to normalised coordinates
            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size

                self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if (point_out.z > 0):
                self.obj_pub.publish(point_out) 
        except CvBridgeError as e:
            print(e)  


def main(args=None):

    rclpy.init(args=args)

    detect_obj = DetectObj()
    while rclpy.ok():
        rclpy.spin_once(detect_obj)
        proc.wait_on_gui()

    detect_obj.destroy_node()
    rclpy.shutdown()

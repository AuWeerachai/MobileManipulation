import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRotateNode(Node):
    def __init__(self):
        super().__init__('image_rotate_node')
        self.bridge = CvBridge()
        # Subscribe to original camera image
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',  
            self.callback,
            10)
        # Publisher for rotated image
        self.pub = self.create_publisher(Image, '/camera/color/image_rotated', 10)

    def callback(self, msg):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Rotate image 90 degrees clockwise
        rotated = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
        # Convert back to ROS Image message
        rotated_msg = self.bridge.cv2_to_imgmsg(rotated, encoding='bgr8')
        rotated_msg.header = msg.header
        # Publish rotated image
        self.pub.publish(rotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageRotateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
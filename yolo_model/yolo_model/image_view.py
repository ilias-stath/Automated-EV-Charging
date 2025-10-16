import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/yolo/annotated',  
            self.listener_callback,
            10)

        # Create a resizable window
        cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)

    def listener_callback(self, msg):
        cv_image = self.ros_image_to_cv2(msg)
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

    def ros_image_to_cv2(self, msg):
        # If it's already a NumPy array, return as-is
        if isinstance(msg, np.ndarray):
            return msg

        # Otherwise, convert ROS Image → NumPy
        dtype = np.uint8 if msg.encoding.endswith('8') else np.uint16
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


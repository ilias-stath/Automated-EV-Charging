import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ShapeDetectorNode(Node):
    def __init__(self):
        super().__init__('shape_detector_node')
        self.get_logger().info('Shape Detector Node has been started.')

        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',  # Replace with your camera's topic name
            self.listener_callback,
            10)

    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # ---- YOUR CODE STARTS HERE ----
        # The detect_rectangle logic goes here, using cv_image instead of reading a file
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            peri = cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, 0.04 * peri, True)

            if len(approx) >= 3:
                rect = cv2.minAreaRect(largest_contour)
                (center_x, center_y), (width, height), angle = rect
                if width < height:
                    angle += 90

                self.get_logger().info(f"Detected a Rectangle! Angle: {angle:.2f} degrees")
                
                # Draw the results for visualization (if needed)
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(cv_image, [box], -1, (0, 0, 255), 2)
                cv2.imshow('Detected Shape', cv_image)
                cv2.waitKey(1) # Use waitKey(1) for live video feeds
            

def main(args=None):
    rclpy.init(args=args)
    shape_detector = ShapeDetectorNode()
    rclpy.spin(shape_detector)
    shape_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
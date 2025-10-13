import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import numpy as np

class CameraInfoReader(Node):
    def __init__(self):
        super().__init__('camera_info_reader')
        self.get_logger().info("Subscribing to /camera/camera_info to get camera intrinsics...")
        # The subscription's quality of service (qos) is set to 1 to receive the latest message
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.listener_callback,
            1)

    def listener_callback(self, msg):
        # The camera intrinsic matrix K is a 3x3 array stored in a 1D list.
        # Reshape it to its correct form.
        K_matrix = np.array(msg.k).reshape(3, 3)
        
        # The distortion coefficients are in a 1D list.
        dist_coeffs = np.array(msg.d)
        
        self.get_logger().info("Camera Intrinsic Matrix (K):")
        self.get_logger().info(f"\n{K_matrix}")
        self.get_logger().info("Distortion Coefficients (D):")
        self.get_logger().info(f"{dist_coeffs}")
        
        # After getting the info, destroy the subscription to prevent it from running again.
        self.destroy_subscription(self.subscription)
        self.get_logger().info("Camera info received and node destroyed.")

def main(args=None):
    rclpy.init(args=args)
    camera_info_reader = CameraInfoReader()
    rclpy.spin(camera_info_reader)
    camera_info_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
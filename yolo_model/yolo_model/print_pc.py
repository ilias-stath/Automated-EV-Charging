import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        self.get_logger().info("Subscribing to /camera/points....")
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.cloud_callback,
            1)
        self.get_logger().info("Point Cloud Processor Node Started!")
        print("\n\n")

        # Flag to track if the node has processed a message
        self.processed_once = False

    def cloud_callback(self, msg):
        self.get_logger().info("Received PointCloud2 message. Processing all points...")

        # Get the dimensions of the point cloud
        cloud_width = msg.width
        cloud_height = msg.height

        # Define the target pixel coordinates
        target_col, target_row = 895, 466

        # Read all points from the PointCloud2 message into a NumPy array.
        # `skip_nans=True` will automatically remove points with invalid coordinates.
        # This approach is more reliable for a one-to-one pixel lookup
        # compared to using the `uvs` argument, which can return multiple points.
        points_xyz = pc2.read_points_numpy(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        )

        self.get_logger().info(f"Read {points_xyz.shape[0]} points.")
        self.get_logger().info(f"Searching for point at pixel ({target_col}, {target_row})...")

        # Find the linear index corresponding to the target pixel coordinates
        linear_index = target_row * cloud_width + target_col

        # Check if the linear index is within the bounds of the point cloud
        if linear_index < points_xyz.shape[0]:
            # Get the point at the calculated index
            point = points_xyz[linear_index]
            
            # Extract the 3D coordinates in meters
            x_m = float(point[0])
            y_m = float(point[1])
            z_m = float(point[2])
            
            self.get_logger().info(
                f"Found point at target! 3D: ({x_m:.3f} m, {y_m:.3f} m, {z_m:.3f} m)"
            )
        else:
            self.get_logger().info(f"Pixel ({target_col}, {target_row}) is outside the point cloud dimensions or has no valid point.")
        
        # Set the flag to true after processing the message
        self.processed_once = True

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    
    # Loop until the node has processed a message and is ready to shut down
    while rclpy.ok() and not node.processed_once:
        rclpy.spin_once(node)
    
    node.destroy_node()
    rclpy.shutdown()

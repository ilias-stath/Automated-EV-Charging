import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image , PointCloud2
from std_msgs.msg import Header
# from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time
import math
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.pub = False
        self.cloud = None
        self.points = []
        self.frame = None
        self.inlets = []
        self.inlet_boxes = []
        # self.bridge = CvBridge()

        # Values from measurement
        self.port_lenght = 130.0
        self.port_width = 90.0
        self.port_dist = 27.0
        self.inlet_inside_lenght = 103.0
        self.port_to_top_dist = 29.5
        self.port_to_bottom_dist = 24.0
        self.ports_up_midle_to_ports_up_top = 11.2
        self.port_to_center_dist = (self.inlet_inside_lenght/2) - self.port_to_bottom_dist
        self.midpoint_up = self.inlet_inside_lenght + self.ports_up_midle_to_ports_up_top - self.port_to_top_dist - self.port_to_bottom_dist
        self.midpoint_up_from_center =  self.midpoint_up - self.port_to_center_dist


        # Subscribing to the image topic
        self.get_logger().info("Subscribing to /camera/image....")
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10)
        
        # Subscribing to the move status topic
        self.get_logger().info("Subscribing to /movement/status....")
        self.status_sub = self.create_subscription(
            String,
            '/movement/status',
            self.status_callback,
            10)
        
        # Subscribing to pointcloud topic
        self.get_logger().info("Subscribing to /camera/points....")
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.cloud_callback,
            10)
        
        # Publishing the status
        self.status_pub = self.create_publisher(String, '/movement/status', 10)
        
        # Publishing the pose
        self.goal_pub = self.create_publisher(Pose, '/movement/goal', 10)

        # Publishing the image
        self.publisher = self.create_publisher(Image, '/yolo/annotated', 10)

        # Load models
        base_path = os.path.dirname(__file__)
        self.get_logger().info("Loading YOLO models...")

        self.model_inlet = YOLO(os.path.join(base_path, 'both_detector.pt'))  # inlet detection model
        self.model_ports = YOLO(os.path.join(base_path, 'dc_detector_better.pt'))  # dc ports detection model

        # self.br = CvBridge()
        self.get_logger().info("YOLO Node Started!")

        print("\n\n")

    def status_callback(self,msg):
        move_status = msg.data
        self.get_logger().info(f"Received move status: {move_status}")
        

        if move_status == "next" and self.cloud is not None and self.frame is not None:

            status_msg = String()
            status_msg.data = "received"
            self.status_pub.publish(status_msg)
            self.get_logger().info("Published status \"Received\" to /movement/status...")

            print("\n\n")
            self.get_logger().info(f"Finding goal position....")
            # rvec = [0.0, 0.0, 0.0]


            tol = 40

            av_row = [] # Get the average distance of point in each row
            inlet_height = 0.0
            inlet_width = 0.0
            

            height, width, _ = self.frame.shape
            self.get_logger().info(f"Frame:  height -> {height}px , width -> {width}px")


            # Center of frame
            # frame_cx = width // 2
            # frame_cy = height // 2

            # Stage 1: Detect inlet
            inlet_results = self.model_inlet(self.frame, verbose=False)

            for inlet_detection in inlet_results:
                annotated = inlet_detection.plot()
                boxes = inlet_detection.boxes.xyxy.cpu().numpy()
                classes = inlet_detection.boxes.cls.cpu().numpy().astype(int)
                names = self.model_inlet.names

                for box, cls in zip(boxes, classes):
                    self.get_logger().info("Found ccs2 inlet....")
                    label = names[cls]
                    if label.lower() == "ccs2-dc":
                        x1, y1, x2, y2 = map(int, box)

                        self.get_logger().info(f"Inlet position: x1 -> {x1}px , y1 -> {y1}px , x2 -> {x2}px , y2 -> {y2}px")

                        inlet_area = (x2-x1)*(y2-y1)

                        self.get_logger().info(f"Inlet area: {inlet_area} px")

                        overlap = False

                        # Loop through the existing boxes with their indices
                        for i, (rx1, ry1, rx2, ry2, r_area) in enumerate(self.inlet_boxes):
                            # Check for overlap between the new box and the current stored box.
                            if self.check_overlap(x1, y1, x2, y2, rx1, ry1, rx2, ry2):
                                # An overlap was found. Now, check the area.
                                print(f"Overlap detected with stored box at index {i}. Stored area: {r_area}, New area: {inlet_area}.")
                                
                                if inlet_area > r_area:
                                    # The new box is larger. Remove the old box and insert the new one.
                                    self.inlet_boxes[i] = [x1, y1, x2, y2, inlet_area]
                                    print("New box is larger. It will replace the stored one.")
                                else:
                                    # The new box is not larger. Do nothing.
                                    print("New box is not larger. It will be discarded.")
                                    overlap = True
                                
                                # In either case, we have found an overlapping box and made a decision.
                                # We can exit the function now.
                                print("Exiting loop.")
                                print("---")
                                break
                            
                        if overlap:
                            continue
                        

                        # If the loop finishes without finding any overlaps, add the new box.
                        print(f"No overlap found. Adding new box with area: {inlet_area}.")
                        self.inlet_boxes.append([x1, y1, x2, y2, inlet_area])

                        # Add tolerance
                        y1t = max(0, y1 - tol)
                        y2t = min(height, y2 + tol)
                        x1t = max(0, x1 - tol)
                        x2t = min(width, x2 + tol)

                        inlet_crop = self.frame[y1t:y2t, x1t:x2t]

                        shape_center_x = 0.0
                        shape_center_y = 0.0
                        angleZ = 0.0
                        angleX_rad = 0.0
                        angleY_rad = 0.0
                        angleZ_rad = 0.0

                        # Find shape and orientation
                        angleZ, shape_center_x, shape_center_y, inlet_width, inlet_height = self.find_shape(inlet_crop, x1t, y1t)


                        port_results = self.model_ports(inlet_crop, verbose=False)

                        port_centers = []
                        for pr in port_results:
                            port_boxes = pr.boxes.xyxy.cpu().numpy()
                            port_classes = pr.boxes.cls.cpu().numpy().astype(int)
                            port_names = self.model_ports.names

                            for pbox, pcls in zip(port_boxes,port_classes):
                                self.get_logger().info(f"Found ccs2 dc port....")
                                # plabel = port_names[pcls]
                                plabel = "CCS2-DC-port"
                                px1, py1, px2, py2 = map(int, pbox)

                                px1, px2 = x1t + px1 , x1t + px2
                                py1, py2 = y1t + py1 , y1t + py2

                                cv2.rectangle(annotated, (px1,py1), (px2,py2), (0, 255, 0), 2)
                                cv2.putText(annotated, plabel, (px1, py1 - 5), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                                cx = (px1 + px2) // 2  
                                cy = (py1 + py2) // 2
                                port_centers.append((cx, cy))
                                self.points.append((cx, cy))
                                cv2.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)
                                self.get_logger().info(f"DC port center at ({cx, cy}) px")

                        # if angleZ > 1.0 or angleZ < -1.0:
                        #     angleZ_rad = math.radians(angleZ)
                        #     self.get_logger().info("Rotation in the x axis required before moving!!")
                        #     self.get_logger().info(f"Anglez in radians: {angleZ_rad:.2f}")
                        #     # pair = [0.0,0.0,0.0,0.0,angleZ_rad,0.0,0.0,0.0]
                        #     # self.inlets.append(pair)
                        #     # self.points.clear()
                        #     # port_centers.clear()

                        if len(port_centers) == 2:

                            # print(port_centers)

                            sorted_ports = sorted(port_centers, key=lambda p: p[0])

                            # print(sorted_ports)

                            # Now, build the final list of 2D points in the correct, fixed order
                            self.points.clear()
                            
                            # Add the sorted DC ports first
                            self.points.extend(sorted_ports) 


                            self.get_logger().info(f"Found 2 ccs2 dc ports....")
                            mid_x = (port_centers[0][0] + port_centers[1][0]) // 2
                            mid_y = (port_centers[0][1] + port_centers[1][1]) // 2
                            cv2.circle(annotated, (mid_x, mid_y), 6, (255, 0, 0), -1)
                            self.points.append((mid_x, mid_y))
                            self.get_logger().info(f"Midpoint at ({mid_x, mid_y}) px")

                            port_pixel_dist = ((port_centers[0][0] - port_centers[1][0])**2 +
                                            (port_centers[0][1] - port_centers[1][1])**2) ** 0.5
                            mm_per_pixel = self.port_dist / port_pixel_dist

                            self.get_logger().info(f"port_pixel_dist/2: {int(port_pixel_dist/4)}")

                            if shape_center_y != 0.0:
                                if shape_center_y < mid_y + int(port_pixel_dist/4):
                                    angleZ = angleZ - 90
                                else:
                                    angleZ = angleZ + 90

                                if angleZ > 180:
                                    angleZ = angleZ - 360

                            self.get_logger().info(f"AngleZ: {angleZ:.2f}")
                            angleZ_rad = math.radians(angleZ)
                            self.get_logger().info(f"AngleZ in radians: {angleZ_rad:.2f}")


                            if angleZ > 0.5 or angleZ < -0.5:
                                self.points.append((shape_center_x, shape_center_y))
                                self.get_logger().info(f"Inlet center at ({shape_center_x, shape_center_y}) px")

                                cv2.circle(annotated, (shape_center_x, shape_center_y), 6, (0, 0, 255), -1)
                                cv2.putText(annotated, "Inlet Center", (shape_center_x + 10, shape_center_y),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                
                                inlet_center_x = shape_center_x
                                inlet_center_y = shape_center_y

                            else:
                                pixel_offset = int(self.midpoint_up / mm_per_pixel)
                                mid_up_x = mid_x
                                mid_up_y = mid_y - pixel_offset
                                cv2.circle(annotated, (mid_up_x, mid_up_y), 6, (255, 0, 0), -1)
                                # self.points.append((mid_up_x, mid_up_y))
                                self.get_logger().info(f"Midpoint up at ({mid_up_x, mid_up_y}) px")

                                pixel_offset = int(self.port_to_center_dist / mm_per_pixel)
                                inlet_center_y = mid_y - pixel_offset
                                inlet_center_x = mid_x

                                self.points.append((inlet_center_x, inlet_center_y))
                                self.get_logger().info(f"Inlet center at ({inlet_center_x, inlet_center_y}) px")

                                cv2.circle(annotated, (inlet_center_x, inlet_center_y), 6, (0, 0, 255), -1)
                                cv2.putText(annotated, "Inlet Center", (inlet_center_x + 10, inlet_center_y),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                            
                            # rvec_raw , tvec_raw = self.get_inlet_pose_pnp(self.points)
                            # self.get_logger().info(f"\n\rvec -> {rvec_raw}  ,  tvec -> {tvec_raw}.")
                            # rvec = rvec_raw.flatten()
                            # tvec = tvec_raw.flatten()

                            # dx_pixel = inlet_center_x - frame_cx
                            # dy_pixel = inlet_center_y - frame_cy
                            # pixel_distance = (dx_pixel*dx_pixel + dy_pixel*dy_pixel) ** 0.5

                            # dx_meter = (dx_pixel*mm_per_pixel) / 1000.0
                            # dy_meter = (dy_pixel*mm_per_pixel) / 1000.0
                            # meter_distance = (pixel_distance*mm_per_pixel) / 1000.0

                        elif shape_center_x != 0.0 and shape_center_y != 0.0:

                            inlet_center_x = shape_center_x
                            inlet_center_y = shape_center_y

                            self.get_logger().info(f"Only inlet detected. Using shape center at ({inlet_center_x}, {inlet_center_y})px as goal.")
                            
                            self.points.append((inlet_center_x, inlet_center_y))

                            cv2.circle(annotated, (inlet_center_x, inlet_center_y), 6, (0, 0, 255), -1)
                            cv2.putText(annotated, "Inlet Center", (inlet_center_x + 10, inlet_center_y),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
  
                        else:
                            inlet_center_x = (x1t + x2t) // 2
                            inlet_center_y = (y1t + y2t) // 2

                            # port_pixel_width = x2t - x1t
                            # port_pixel_lenght = y2t - y1t

                            # mm_per_pixel_width = port_width / port_pixel_width
                            # mm_per_pixel_length = port_lenght / port_pixel_lenght

                            # mm_per_pixel = (mm_per_pixel_width + mm_per_pixel_length) / 2

                            self.get_logger().info(f"Only inlet detected. Using bounding box center at ({inlet_center_x}, {inlet_center_y})px as goal.")
                            
                            self.points.append((inlet_center_x, inlet_center_y))

                            cv2.circle(annotated, (inlet_center_x, inlet_center_y), 6, (0, 0, 255), -1)
                            cv2.putText(annotated, "Inlet Center", (inlet_center_x + 10, inlet_center_y),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                            
                            # dx_pixel = inlet_center_x - frame_cx
                            # dy_pixel = inlet_center_y - frame_cy
                            # pixel_distance = (dx_pixel*dx_pixel + dy_pixel*dy_pixel) ** 0.5

                            # dx_meter = (dx_pixel*mm_per_pixel) / 1000.0
                            # dy_meter = (dy_pixel*mm_per_pixel) / 1000.0
                            # meter_distance = (pixel_distance*mm_per_pixel) / 1000.0
                        if len(self.points) > 0:
                            self.pub = True
                            self.get_logger().info(f" Points -> {self.points}")
                            cloud_width = self.cloud.width

                            points_xyz = pc2.read_points_numpy(
                                self.cloud,
                                field_names=["x", "y", "z"],
                                skip_nans=True,
                            )
                        
                            self.get_logger().info("Got the points....")
                            x, y, z = 0.0, 0.0, 0.0


                            # if angleZ > 0.5 or angleZ < -0.5:
                            if False: #For now it is off
                                x22 = (inlet_center_x + inlet_height/2)*math.cos(abs(angleZ_rad)) + (inlet_center_y + inlet_height/2)*math.sin(abs(angleZ_rad))
                                y22 = (inlet_center_y)*math.cos(abs(angleZ_rad)) + (inlet_center_x)*math.sin(abs(angleZ_rad))
                                x33 = (inlet_center_x)*math.cos(abs(angleZ_rad)) + (inlet_center_y)*math.sin(abs(angleZ_rad))
                                y33 = (inlet_center_y + inlet_width/2)*math.cos(abs(angleZ_rad)) + (inlet_center_x + inlet_width/2)*math.sin(abs(angleZ_rad))
                                self.get_logger().info(f"x2 -> {x22}  ,  y2 -> {y22} ")
                                self.get_logger().info(f"x3 -> {x33}  ,  y2 -> {y33} ")
                                cv2.circle(annotated, (int(x22), int(y22)), 6, (0, 0, 255), -1)
                                cv2.putText(annotated, "(x2,y2)", (int(x22) + 10, int(y22)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                cv2.circle(annotated, (int(x33), int(y33)), 6, (0, 0, 255), -1)
                                cv2.putText(annotated, "(x33,y33)", (int(x33) + 10, int(y33)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                                    

                            else:
                                # First find tilts
                                # Iterate for every 5 rows in bounding box
                                av_row.clear()
                                for row in range(y1+20,y2-20,5):
                                    rowAvX = 10.0
                                    rowAvZ = 10.0
                                    rowAvY = 10.0
                                    # index = 0
                                    # Iterate for every col in bounding box that is close to the center
                                    for col in range(inlet_center_x-5,inlet_center_x+5,1):
                                        linear_index = row * cloud_width + col
                                        if linear_index < points_xyz.shape[0]:
                                            point = points_xyz[linear_index]
                                            x = float(point[0])
                                            y = float(point[1])
                                            z = float(point[2])
                                            if rowAvX > x:
                                                rowAvX = x
                                                rowAvY = y
                                                rowAvZ = z
                                            # index += 1
                                            # self.get_logger().info(f"Pixel ({col}, {row}) at dist -> {x}.")
                                        else:
                                            self.get_logger().info(f"Pixel ({col}, {row}) is outside the point cloud dimensions or has no valid point.")
                                    # rowAvX = rowAvX/index
                                    # self.get_logger().info(f"row = {row}  ,  rowAvX = {rowAvX}  ,  rowAvY = {rowAvY}  ,  rowAvZ = {rowAvZ}")
                                    pair = [rowAvX,rowAvY,rowAvZ]
                                    av_row.append(pair)
                                    

                                    # input("Press Enter to continue to the next row...")

                                # Get angles for x and y
                                angleX_rad = self.find_orientation_x(av_row)
                                av_row.clear()


                                for col in range(x1+40,x2-40,5):
                                    rowAvX = 10.0
                                    rowAvZ = 10.0
                                    rowAvY = 10.0
                                    # index = 0
                                    # Iterate for every col in bounding box that is close to the center
                                    for row in range(inlet_center_y-5,inlet_center_y+5,1):
                                        linear_index = row * cloud_width + col
                                        if linear_index < points_xyz.shape[0]:
                                            point = points_xyz[linear_index]
                                            x = float(point[0])
                                            y = float(point[1])
                                            z = float(point[2])
                                            if rowAvX > x:
                                                rowAvX = x
                                                rowAvY = y
                                                rowAvZ = z
                                            # index += 1
                                            # self.get_logger().info(f"Pixel ({col}, {row}) at dist -> {x}.")
                                        else:
                                            self.get_logger().info(f"Pixel ({col}, {row}) is outside the point cloud dimensions or has no valid point.")
                                    # rowAvX = rowAvX/index
                                    # self.get_logger().info(f"row = {col}  ,  rowAvX = {rowAvX}  ,  rowAvY = {rowAvY}  ,  rowAvZ = {rowAvZ}")
                                    pair = [rowAvX,rowAvY,rowAvZ]
                                    av_row.append(pair)

                                angleY_rad = self.find_orientation_y(av_row)
                                av_row.clear()


                            self.get_logger().info(f"angleX_rad = {angleX_rad}  ,  angleY_rad = {angleY_rad}")
                            # self.get_logger().info(f"angleX_rad = {angleX_rad}  ,  angleY_rad = {angleY_rad}")
                            
                            # Then find center etc
                            for target_col,target_row in self.points:
                                linear_index = target_row * cloud_width + target_col
                                if linear_index < points_xyz.shape[0]:
                                    point = points_xyz[linear_index]
                                    x = float(point[0])
                                    y = float(point[1])
                                    z = float(point[2])
                                    if target_col == inlet_center_x and target_row == inlet_center_y:
                                        point = np.array([x, y, z])
                                        dist = np.linalg.norm(point)
                                        if dist < 1.2:
                                            pair = [x,y,z,angleX_rad,angleY_rad,angleZ_rad,0.0,dist]
                                            self.inlets.append(pair)
                                            self.get_logger().info(f"Placed inlet center of pixel ({target_col}, {target_row}) and of 3D location ({x:.4f} m, {y:.4f} m, {z:.4f} m) at list!")

                                    self.get_logger().info(f"Found pixel point ({target_col}, {target_row}) at 3D location: ({x:.4f} m, {y:.4f} m, {z:.4f} m)")
                                else:
                                    self.get_logger().info(f"Pixel ({target_col}, {target_row}) is outside the point cloud dimensions or has no valid point.")

                        
                        
                        else:
                            self.get_logger().info("No inlet center..")


            if len(self.inlets) > 0:

                for x,y,z,qx,qy,qz,qw,dist in self.inlets:
                    self.get_logger().info(f"Point -> x={x} m , y={y} m , z={z} m , qx={qx} m , qy={qy} m , qz={qz} m , qw={qw} m , dist={dist} m")


                    pose_msg = Pose()
                    pose_msg.position.x = -1*x
                    pose_msg.position.y = -y
                    pose_msg.position.z = -z #+ 0.05  
                    pose_msg.orientation.x = qx
                    pose_msg.orientation.y = -qy
                    pose_msg.orientation.z = qz
                    pose_msg.orientation.w = qw

                    self.goal_pub.publish(pose_msg)
                    self.get_logger().info("Published goal to /movement/goal")

                    # print(f"Offset X: {dx_pixel} px  ,  Offset Y: {dy_pixel} px  ,  Distance: {pixel_distance:.2f} px")
                    # print(f"Offset X: {dx_meter:.4f} m  ,  Offset Y: {dy_meter:.4f} m  ,  Distance: {meter_distance:.4f} m")

                # Add a delay to be sure that goals are received
                self.get_logger().info("Waiting 2 seconds before publishing 'sent' status...")
                self.get_clock().sleep_for(Duration(seconds=2))
                status_msg = String()
                status_msg.data = "sent"
                self.status_pub.publish(status_msg)
                self.get_logger().info("Published status \"sent\" to /movement/status...")

                # annotated_msg = self.br.cv2_to_imgmsg(annotated, encoding='bgr8')
                annotated_msg = self.ros_image_to_cv2(annotated)
                # annotated_msg.header = Header()
                annotated_msg = self.cv2_to_ros_image(annotated)
                annotated_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(annotated_msg)
                self.get_logger().info("Published image to /yolo/annotated...")

            else:
                self.get_logger().info("No inlets detected")
                status_msg = String()
                status_msg.data = "nothing"
                self.status_pub.publish(status_msg)
                self.get_logger().info("Published status \"nothing\" to /movement/status...")

            self.points.clear()
            self.inlets.clear()
            self.inlet_boxes.clear()

        else:
            self.get_logger().info(f"Robot not yet ready or no point cloud or no frame....")



    def cloud_callback(self, msg):
        self.cloud = msg


    def listener_callback(self, data):
        # self.frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.frame = self.ros_image_to_cv2(data)



    # def get_inlet_pose_pnp(self, image_points):

    #     if len(image_points) < 4:
    #         self.get_logger().error("PnP requires at least 4 points to solve for pose.")
    #         return None, None

    #     # Ensure points are numpy arrays of the correct type
    #     obj_points = self.inlet_3d_points
    #     img_points = np.array(image_points, dtype=np.float32)

    #     try:
    #         # Solve PnP
    #         success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
            
    #         if success:
    #             self.get_logger().info("PnP solved successfully.")
    #             return rvec, tvec
    #         else:
    #             self.get_logger().error("PnP failed to find a valid pose.")
    #             return None, None

    #     except cv2.error as e:
    #         self.get_logger().error(f"OpenCV PnP error: {e}")
    #         return None, None


    # def is_contained_in(self, box_a, box_b):
    #     """Checks if box_a is completely contained within box_b."""
    #     x1_a, y1_a, x2_a, y2_a = box_a
    #     x1_b, y1_b, x2_b, y2_b = box_b
    #     return (x1_a >= x1_b and y1_a >= y1_b and x2_a <= x2_b and y2_a <= y2_b)

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

    def cv2_to_ros_image(self, img, frame_id="camera"):
        msg = Image()
        msg.header = Header()
        msg.header.frame_id = frame_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg


    def find_shape(self, image, offset_x, offset_y):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 0)
        edged = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            peri = cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, 0.04 * peri, True)
            self.get_logger().info(f"Approx: {approx}")
            
            if len(approx) >= 3:
                rect = cv2.minAreaRect(largest_contour)
                (center_x, center_y), (width, height), angle = rect

                center_x = int(center_x)
                center_y = int(center_y)

                # Calculate the image center
                frame_height, frame_width, _ = image.shape
                image_center_x = frame_width // 2
                image_center_y = frame_height // 2
                self.get_logger().info(f"Image center at ({image_center_x},{image_center_y}) px")
                self.get_logger().info(f"Inlet_width -> {width}  ,  inlet_height -> {height} px")


                # Define tolerance
                tolerance = 50 


                area = width*height

                if width < height:
                    angle += 90

                # Calculate the absolute center coordinates by adding the offsets
                absolute_center_x = int(center_x) + offset_x
                absolute_center_y = int(center_y) + offset_y

                self.get_logger().info(f"Detected a Rectangle! Angle: {angle:.2f} degrees, Area: {area} px")
                self.get_logger().info(f"Detected center at ({absolute_center_x},{absolute_center_y}) px")
                
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.circle(image, (center_x, center_y), 6, (0, 0, 255), -1)
                cv2.drawContours(image, [box], -1, (0, 0, 255), 2)
                # cv2.imshow('Detected Shape', image)
                # cv2.waitKey(0)

                # Check if the shape is at the center
                if abs(center_x - image_center_x) < tolerance and abs(center_y - image_center_y) < tolerance:
                    self.get_logger().info("The shape is at the center of the image.")
                    return angle, absolute_center_x, absolute_center_y, width, height
                else:
                    self.get_logger().info("The shape is NOT at the center of the image.")
                    return 0.0, 0.0, 0.0, 0.0, 0.0
                
            else:
                self.get_logger().info("No shape??")

        return 0.0, 0.0, 0.0, 0.0, 0.0
    

    def check_overlap(self, rect1_x1, rect1_y1, rect1_x2, rect1_y2, rect2_x1, rect2_y1, rect2_x2, rect2_y2):

        if max(rect1_x1,rect2_x1) < min(rect1_x2,rect2_x2) and max(rect1_y1,rect2_y1) < min(rect1_y2,rect2_y2):
            return True
        else:
            return False


    def find_orientation_x(self,rowAv):
        # last = len(rowAv) - 1
        a = 0.0
        b = 0.0
        c = 0.0
        th = 0.0
        # count = 0
        
        # # Check for consistency in the first and last 5 values to get better results and avoid errors 
        # for i in range(1,6):
        #     if(rowAv[0][0] > rowAv[0][len-1]):
        #         if(rowAv[0][0] < rowAv[0][i]):
        #             count = count + 1
        #         else:
        #             count = 0
        #     else:


        # I basically create a triangle from known values to get the angle
        print(f"\n\nrowAv[-1][0] -> {rowAv[-1][0]}  ,  rowAv[0][0]-> {rowAv[0][0]}  ,  rowAv[-1][2] -> {rowAv[-1][2]}  ,  rowAv[0][2] -> {rowAv[0][2]}\n\n")
        a = rowAv[-1][0] - rowAv[0][0] # From the difference in distance from camera I get 1 side
        b = abs(rowAv[-1][2] - rowAv[0][2]) # From the difference in distance from start to finish I get 1 side
        c = (a**2 + b**2)**0.5 # From pythagoral theorem I get the other

        th = math.atan(a/b)
        

        self.get_logger().info(f"a -> {a}  ,  b -> {b}  ,  c -> {c}  ,  thX -> {th}")

        return th
    
    
    def find_orientation_y(self,rowAv):
        # last = len(rowAv) - 1
        a = 0.0
        b = 0.0
        c = 0.0
        th = 0.0
        # count = 0
        
        # # Check for consistency in the first and last 5 values to get better results and avoid errors 
        # for i in range(1,6):
        #     if(rowAv[0][0] > rowAv[0][len-1]):
        #         if(rowAv[0][0] < rowAv[0][i]):
        #             count = count + 1
        #         else:
        #             count = 0
        #     else:


        # I basically create a triangle from known values to get the angle
        print(f"\n\nrowAv[-1][0] -> {rowAv[-1][0]}  ,  rowAv[0][0]-> {rowAv[0][0]}  ,  rowAv[-1][1] -> {rowAv[-1][1]}  ,  rowAv[0][1] -> {rowAv[0][1]}\n\n")
        a = rowAv[-1][0] - rowAv[0][0] # From the difference in distance from camera I get 1 side
        b = abs(rowAv[-1][1] - rowAv[0][1]) # From the difference in distance from start to finish I get 1 side
        c = (a**2 + b**2)**0.5 # From pythagoral theorem I get the other

        th = math.atan(a/b)
        

        self.get_logger().info(f"a -> {a}  ,  b -> {b}  ,  c -> {c}  ,  thX -> {th}")

        return th





            


            
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


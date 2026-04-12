#!/usr/bin/env python3

import os
import time
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

# ROS2 Messages
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Deep Learning & Math
import tensorflow as tf
from keras.models import load_model
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation, binary_opening

# Import your planner (Ensure rrt_planner_b.py is in the same directory)
from perception_pkg.rrt_planner_b import BidirectionalRRT

class UGVVisionNavigator(Node):
    def __init__(self):
        super().__init__('ugv_vision_navigator')
        self.get_logger().info("--- [SYSTEM] Booting UGV Vision Navigation Node ---")

        # --- TUNING PARAMETERS ---
        self.input_shape = (224, 224)
        self.safety_margin = 2      # Pixel inflation radius for 224x224 map
        self.rrt_step_size = 15
        self.rrt_max_iter = 5000
        
        # --- CAMERA INTRINSICS (From your teammate) ---
        self.fx = 673.364213244571
        self.fy = 673.364213244571
        self.cx = 640.5
        self.cy = 360.5
        self.Z = 10.0  # Altitude in meters. (Ideally, subscribe to the drone's altitude topic later!)
        
        # --- ROS2 SETUP ---
        self.bridge = CvBridge()
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/drone_camera/image_raw', # Check your gazebo topic!
            self.image_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/ugv/planned_path', 10)

        # --- LOAD NEURAL NETWORK ---
        # CHANGE THIS PATH to point to your actual .keras file
        model_path = os.path.expanduser("~/grp/src/perception_pkg/perception_pkg/best_model.h5")
        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ Model missing at {model_path}")
            raise FileNotFoundError("Model file not found.")
            
        self.get_logger().info(f"[*] Loading Perception Model...")
        self.model = load_model(model_path, compile = False)
        self.get_logger().info("[*] Node Initialized and Ready.")

    def pixel_to_world(self, u, v):
        """ Teammate's Helper: Calculates 3D metric position from a high-res pixel. """
        X = ((u - self.cx) * self.Z) / self.fx
        Y = ((v - self.cy) * self.Z) / self.fy
        return X, Y

    def detect_endpoints(self, cv_image):
        """ Uses HSV thresholding to find Blue (UGV) and Red (Target). """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Blue
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Red (Wraps around HSV scale)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        start_xy = self.get_centroid(mask_blue)
        goal_xy = self.get_centroid(mask_red)
        return start_xy, goal_xy

    def get_centroid(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    def smooth_path_bspline(self, path_list, num_points=50):
        if not path_list or len(path_list) < 3:
            return np.array(path_list) if path_list else None
        path_array = np.array(path_list)
        x, y = path_array[:, 0], path_array[:, 1]
        try:
            tck, u = splprep([x, y], s=5.0, k=3)
            u_new = np.linspace(0, 1, num_points)
            x_new, y_new = splev(u_new, tck)
            return np.column_stack((x_new, y_new))
        except Exception as e:
            self.get_logger().warn(f"Smoothing failed: {e}")
            return path_array

    def image_callback(self, msg):
        t0 = time.time()
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # 1. Detect Endpoints on the HIGH-RES image (e.g., 1280x720)
        start_xy_highres, goal_xy_highres = self.detect_endpoints(cv_image)
        if not start_xy_highres or not goal_xy_highres:
            self.get_logger().info("Waiting for UGV(Blue) and Target(Red) in view...", throttle_duration_sec=2.0)
            return

        # 2. Scale coordinates down to the 224x224 RRT grid
        orig_height, orig_width = cv_image.shape[:2]
        scale_x = 224.0 / orig_width
        scale_y = 224.0 / orig_height

        start_node = (int(start_xy_highres[1] * scale_y), int(start_xy_highres[0] * scale_x)) # (row, col)
        goal_node = (int(goal_xy_highres[1] * scale_y), int(goal_xy_highres[0] * scale_x))

        # 3. Prepare Image for Neural Network
        cv_image_resized = cv2.resize(cv_image, self.input_shape)
        rgb_image = cv2.cvtColor(cv_image_resized, cv2.COLOR_BGR2RGB)
        img_tensor = np.expand_dims((rgb_image / 255.0).astype(np.float32), axis=0)

        # 4. Neural Network Inference
        prediction = self.model.predict(img_tensor, verbose=0)
        pred_mask = np.argmax(prediction, axis=-1)[0]

        # 5. Create Safety Map
        traversable_map = np.zeros_like(pred_mask)
        traversable_map[pred_mask == 2] = 1 # High Veg
        traversable_map[pred_mask == 3] = 1 # Trees
        traversable_map[pred_mask == 4] = 1 # Obstacles

        clean_map = binary_opening(traversable_map, structure=np.ones((3,3))).astype(np.int32)
        nav_map = binary_dilation(clean_map, iterations=self.safety_margin).astype(np.int32)

        if nav_map[start_node[0], start_node[1]] == 1 or nav_map[goal_node[0], goal_node[1]] == 1:
            self.get_logger().warn("Start/Goal inside obstacle. Aborting plan.", throttle_duration_sec=2.0)
            return

        # 6. Bi-RRT Planning on 224x224 Map
        rrt = BidirectionalRRT(nav_map, start_node, goal_node, self.rrt_step_size, self.rrt_max_iter)
        raw_path = rrt.plan()
        
        if raw_path:
            smooth_path_224 = self.smooth_path_bspline(raw_path)
            
            # 7. Convert Path to Metric 3D using Teammate's Math
            path_msg = Path()
            path_msg.header.frame_id = "camera_link"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for pt in smooth_path_224:
                # Scale back up to high-res
                u_highres = pt[0] / scale_x
                v_highres = pt[1] / scale_y

                # Apply Pinhole Math
                X, Y = self.pixel_to_world(u_highres, v_highres)

                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(X)
                pose.pose.position.y = float(-Y) # Matching your teammate's axis logic
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            
            latency = time.time() - t0
            self.get_logger().info(f"✅ Path Published! ({len(path_msg.poses)} pts) | Latency: {latency:.2f}s")
        else:
            self.get_logger().warn("❌ RRT failed to find a path.")

def main(args=None):
    rclpy.init(args=args)
    node = UGVVisionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
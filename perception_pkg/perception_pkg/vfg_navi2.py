#!/usr/bin/env python3

import os
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

# ROS2 Messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

# Deep Learning
import tensorflow as tf
from tensorflow.keras.models import load_model
from scipy.ndimage import binary_dilation, binary_opening

class UGVVisionNavigator(Node):
    def __init__(self):
        super().__init__('vfg_navi')
        self.get_logger().info("--- [SYSTEM] Booting UGV VFG+APF Navigation Node ---")

        # --- TUNING PARAMETERS (VFG & APF) ---
        self.k_vfg = 2          # VFG Convergence gain
        self.k_apf = 1         # APF Repulsion gain
        self.safe_dist = 0.8      # Distance (meters) APF starts acting
        self.v_max = 0.7          # Max linear velocity (m/s)
        self.k_omega = 0.5        # Proportional gain for yaw control
        
        self.input_shape = (224, 224)
        self.safety_margin = 5     # Pixel inflation radius
        
        # --- MAP REFRESH LOGIC ---
        self.last_map_update_time = 0.0
        self.map_refresh_interval = 20.0  # Seconds
        self.latest_base_map = None       
        self.obstacles_px_data = []
        
        # --- CAMERA INTRINSICS ---
        self.fx = 673.364213244571
        self.fy = 673.364213244571
        self.cx = 640.5
        self.cy = 360.5
        self.Z = 15.0  
        
        # --- STATE VARIABLES ---
        self.robot_yaw = None          # Heading from MATLAB Odometry
        self.visual_start = None       # Current [x, y] of UGV from camera
        self.fixed_path_start = None   # STATIC [x, y] of UGV when node started (For VFG Line)
        self.target_world = None       # Current [x, y] of Target from camera
        self.obstacles_world = []      # List of [x, y, radius] from NN mask
        
        # --- ROS2 SETUP ---
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)
        
        self.image_sub = self.create_subscription(Image, '/drone_camera/image_raw', self.image_callback, qos)
        self.odom_sub = self.create_subscription(Pose, '/ssmr_position_states_for_matlab', self.odom_callback, qos)
        
        # NOTE: Verify this is the correct topic for your UGV to move!
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', qos)

        # Control Loop Timer (Runs at 10 Hz)
        self.create_timer(0.1, self.control_loop)

        # --- LOAD NEURAL NETWORK ---
        model_path = os.path.expanduser("~/grp/src/perception_pkg/perception_pkg/best_model.h5")
        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ Model missing at {model_path}")
            raise FileNotFoundError("Model file not found.")
            
        self.model = load_model(model_path)
        self.get_logger().info("[*] Node Initialized and Ready.")

    # ==========================================
    #             HELPER FUNCTIONS
    # ==========================================
    def pixel_to_world(self, u, v):
        X = ((u - self.cx) * self.Z) / self.fx
        Y = ((v - self.cy) * self.Z) / self.fy
        return X, Y

    def get_centroid(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] == 0: return None
        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    def detect_endpoints(self, hsv):
        # 1. UGV Detection (Blue)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        start_xy = self.get_centroid(mask_blue)

        # 2. Target Detection (Red)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        goal_xy = self.get_centroid(mask_red)

        return start_xy, goal_xy

    # ==========================================
    #             ROS CALLBACKS
    # ==========================================
    def odom_callback(self, msg: Pose):
        # Extract yaw from the quaternion correctly
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            return

        current_time = time.time()
        orig_height, orig_width = cv_image.shape[:2]
        scale_x = 224.0 / orig_width
        scale_y = 224.0 / orig_height

        # --- 1. CONTINUOUS TRACKING ---
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        start_xy_highres, goal_xy_highres = self.detect_endpoints(hsv)
        
        if not start_xy_highres or not goal_xy_highres:
            self.get_logger().info("VISION WAIT: Cannot see UGV or Target in camera feed.", throttle_duration_sec=3.0)
            return

        # Map to World Coordinates
        X_start, Y_start = self.pixel_to_world(start_xy_highres[0], start_xy_highres[1])
        X_goal, Y_goal = self.pixel_to_world(goal_xy_highres[0], goal_xy_highres[1])
        
        # Update current state
        self.visual_start = [X_start, -Y_start]
        self.target_world = [X_goal, -Y_goal]

        # Lock in the fixed path start point for VFG math
        if self.fixed_path_start is None:
            self.fixed_path_start = [X_start, -Y_start]
            self.get_logger().info(f"Locked VFG Start Point: {self.fixed_path_start}")

        start_node = (int(start_xy_highres[1] * scale_y), int(start_xy_highres[0] * scale_x))
        goal_node = (int(goal_xy_highres[1] * scale_y), int(goal_xy_highres[0] * scale_x))

        # --- 2. SEMI-DYNAMIC MAP REFRESH (20 seconds) ---
        if current_time - self.last_map_update_time >= self.map_refresh_interval:
            self.get_logger().info("--- Refreshing Obstacle Map via Neural Network ---")
            
            cv_image_resized = cv2.resize(cv_image, self.input_shape)
            rgb_image = cv2.cvtColor(cv_image_resized, cv2.COLOR_BGR2RGB)
            img_tensor = np.expand_dims((rgb_image / 255.0).astype(np.float32), axis=0)
            
            prediction = self.model.predict(img_tensor, verbose=0)
            pred_mask = np.argmax(prediction, axis=-1)[0]

            traversable_map = np.zeros_like(pred_mask)
            traversable_map[pred_mask == 2] = 1 
            traversable_map[pred_mask == 3] = 1 
            traversable_map[pred_mask == 4] = 1 

            clean_map = binary_opening(traversable_map, structure=np.ones((3,3))).astype(np.uint8)
            nav_map = binary_dilation(clean_map, iterations=self.safety_margin).astype(np.uint8)

            cv2.circle(nav_map, (start_node[1], start_node[0]), 15, 0, -1)
            cv2.circle(nav_map, (goal_node[1], goal_node[0]), 15, 0, -1)

            contours, _ = cv2.findContours(nav_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            new_obstacles = []
            new_obstacles_px = []

            safe_dist_px = self.safe_dist * (self.fx / self.Z) * scale_x

            for c in contours:
                (x_px, y_px), radius_px = cv2.minEnclosingCircle(c)
                
                u_highres = x_px / scale_x
                v_highres = y_px / scale_y
                X_obs, Y_obs = self.pixel_to_world(u_highres, v_highres)
                r_world = (radius_px / scale_x) * self.Z / self.fx
                
                new_obstacles.append([X_obs, -Y_obs, r_world])
                new_obstacles_px.append((int(x_px), int(y_px), int(radius_px), int(safe_dist_px)))
                
            self.obstacles_world = new_obstacles
            self.obstacles_px_data = new_obstacles_px
            
            self.latest_base_map = cv2.cvtColor(nav_map * 255, cv2.COLOR_GRAY2BGR)
            self.last_map_update_time = current_time

        # --- 3. VISUALIZATION ---
        if self.latest_base_map is not None:
            viz_map = self.latest_base_map.copy()
            
            # Dynamic markers
            cv2.circle(viz_map, (start_node[1], start_node[0]), 5, (0, 255, 0), -1) 
            cv2.circle(viz_map, (goal_node[1], goal_node[0]), 5, (0, 0, 255), -1)
            
            # Obstacles
            for (ox, oy, r, r_safe) in self.obstacles_px_data:
                cv2.circle(viz_map, (ox, oy), r + r_safe, (0, 255, 255), 1) # Yellow safe boundary
                cv2.circle(viz_map, (ox, oy), r, (0, 0, 255), 2)            # Red physical boundary

            cv2.imshow("Neural Nav Map", viz_map)
            cv2.waitKey(1)

    # ==========================================
    #             CONTROL LOOP (10 Hz)
    # ==========================================
    def control_loop(self):
        # Check if we have all necessary data to move
        if self.visual_start is None or self.target_world is None or self.fixed_path_start is None:
            return
        if self.robot_yaw is None:
            self.get_logger().info("ODOM WAIT: Waiting for Yaw from /odom", throttle_duration_sec=3.0)
            return

        # Use Camera Coordinates for X,Y position, and Odometry for Yaw
        xr, yr = self.visual_start  
        theta = self.robot_yaw
        
        x1, y1 = self.fixed_path_start
        x2, y2 = self.target_world

        # Check Goal
        dist_to_goal = math.sqrt((xr - x2)**2 + (yr - y2)**2)
        if dist_to_goal < 0.3:
            self.stop_robot()
            self.get_logger().info("✅ Goal Reached!", throttle_duration_sec=2.0)
            # Reset path start so it recalculates a new line if target moves
            self.fixed_path_start = None 
            return

        # 1. VFG (Attractive Force)
        dx = x2 - x1
        dy = y2 - y1
        chi_p = math.atan2(dy, dx)
        
        error_cross = (xr - x1)*math.sin(chi_p) - (yr - y1)*math.cos(chi_p)
        chi_err = math.atan(self.k_vfg * error_cross)
        psi_vfg = chi_p - chi_err
        
        v_vfg_x = self.v_max * math.cos(psi_vfg)
        v_vfg_y = self.v_max * math.sin(psi_vfg)

        # 2. APF (Repulsive Force)
        f_apf_x, f_apf_y = 0.0, 0.0
        
        for obs in self.obstacles_world:
            ox, oy, r_obs = obs
            dist = math.sqrt((xr - ox)**2 + (yr - oy)**2)
            
            # max() prevents divide-by-zero if robot touches exact edge
            dist_surface = max(dist - r_obs, 0.01) 
            
            if dist_surface < self.safe_dist:
                mag = self.k_apf * (1.0/dist_surface - 1.0/self.safe_dist) * (1.0/(dist_surface**2))
                
                vec_x = xr - ox
                vec_y = yr - oy
                vec_len = max(math.sqrt(vec_x**2 + vec_y**2), 0.01)
                
                f_apf_x += (vec_x/vec_len) * mag
                f_apf_y += (vec_y/vec_len) * mag

        # 3. Combine Vectors
# Invert the total vector direction before calculating desired_yaw
        total_x = v_vfg_x + f_apf_x
        total_y = v_vfg_y + f_apf_y

        # 4. Kinematics Output
        desired_yaw = math.atan2(total_y, total_x)
        
        v_cmd = self.v_max

        # Yaw error normalized [-pi, pi]
        yaw_err = desired_yaw - theta
        while yaw_err > math.pi: yaw_err -= 2*math.pi
        while yaw_err < -math.pi: yaw_err += 2*math.pi

        # Create and Publish Twist
        cmd = Twist()
        speed_scaling = max(0.0, math.cos(yaw_err))
        safe_v_cmd = v_cmd * speed_scaling # scale the speed down if looking at a different direction
        # cmd.linear.x = float(safe_v_cmd)
        # cmd.angular.z = float(self.k_omega * yaw_err)
        
        self.cmd_pub.publish(cmd)
        
        # Debug Print to prove loop is running
        self.get_logger().info(f"CMD -> v: {v_cmd:.2f}, w: {cmd.angular.z:.2f} | Dist: {dist_to_goal:.1f}m\n"
                               + 
                               f"\nrobot yaw: {self.robot_yaw:.3f}, \ndes yaw: {desired_yaw:.3f}"
                               +
                               f"\nerror_cross: {error_cross:.3f}", throttle_duration_sec=0.5)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = UGVVisionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
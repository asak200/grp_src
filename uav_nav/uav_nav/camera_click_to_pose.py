#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2

class CameraClickToPose(Node):
    def __init__(self):
        super().__init__('camera_click_to_pose')
        
        self.bridge = CvBridge()
        
        # Subscriptions and Publishers
        self.subscription = self.create_subscription(
            Image,
            '/drone_camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Pose, '/waypoint', 10)
        
        # Camera Intrinsics
        self.fx = 673.364213244571
        self.fy = 673.364213244571
        self.cx = 640.5
        self.cy = 360.5
        self.Z = 10.0 
        
        # Tracking variables
        self.mouse_x, self.mouse_y = 0, 0
        self.latest_image = None
        
        # OpenCV Setup
        self.window_name = "Drone Camera (Hover to see coords, Click to send)"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.timer = self.create_timer(0.033, self.display_callback)
        self.get_logger().info("Node started. Move mouse over the window to see 3D coordinates.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def pixel_to_world(self, u, v):
        """ Helper to calculate 3D position from pixel """
        X = ((u - self.cx) * self.Z) / self.fx
        Y = ((v - self.cy) * self.Z) / self.fy
        return X, Y

    def mouse_callback(self, event, x, y, flags, param):
        # Update current mouse position for the overlay
        self.mouse_x, self.mouse_y = x, y
        
        if event == cv2.EVENT_LBUTTONDOWN:
            X, Y = self.pixel_to_world(x, y)
            
            # Create and publish message
            pose_msg = Pose()
            pose_msg.position.x = X
            pose_msg.position.y = -Y
            pose_msg.position.z = 0.0
            pose_msg.orientation.w = 1.0
            
            self.publisher.publish(pose_msg)
            self.get_logger().info(f"Published Pose: X={X:.2f}, Y={Y:.2f}")

    def display_callback(self):
        if self.latest_image is not None:
            # Create a copy so we don't draw on the raw data (optional but cleaner)
            display_img = self.latest_image.copy()
            
            # Calculate 3D position for the current mouse location
            X_hover, Y_hover = self.pixel_to_world(self.mouse_x, self.mouse_y)
            
            # Prepare text strings
            pixel_text = f"Pixel: [{self.mouse_x}, {self.mouse_y}]"
            world_text = f"World (m): X={X_hover:.2f}, Y={Y_hover:.2f}, Z={self.Z:.1f}"
            
            # Draw a crosshair at the mouse position
            cv2.drawMarker(display_img, (self.mouse_x, self.mouse_y), (0, 255, 0), 
                           cv2.MARKER_CROSS, 20, 2)
            
            # Draw background rectangle for readability
            cv2.rectangle(display_img, (10, 10), (450, 80), (0, 0, 0), -1)
            
            # Write text on screen
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(display_img, pixel_text, (20, 35), font, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(display_img, world_text, (20, 65), font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
            
            cv2.imshow(self.window_name, display_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraClickToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
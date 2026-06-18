#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge

class UsbCamPublisher(Node):

    def __init__(self):
        super().__init__('real_camera_driver')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        timer_period = 1.0 / 30.0  # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize OpenCV Video Capture
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device. Check connection/index.')
            
        self.br = CvBridge()
        
        # Cache standard camera info placeholders based on default frame dims
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
        self.camera_info_msg = self.get_default_camera_info()
        
        self.get_logger().info('real_camera_driver node initialized')

    def get_default_camera_info(self):
        """Generates the ACCURATE calibrated CameraInfo message."""
        info = CameraInfo()
        info.width = 640
        info.height = 480
        info.distortion_model = "plumb_bob"
        
        # Distortion coefficients [k1, k2, t1, t2, k3]
        info.d = [0.072630, -0.138369, 0.006380, -0.002930, 0.000000]
        
        # Intrinsics Matrix K 
        # [fx,  0, cx,
        #   0, fy, cy,
        #   0,  0,  1]
        info.k = [
            615.770802,   0.000000, 319.666337,
              0.000000, 622.158350, 265.606275,
              0.000000,   0.000000,   1.000000
        ]
        info.k = [float(x) for x in info.k]
        
        # Rectification Matrix R (Identity matrix)
        info.r = [
            1.000000, 0.000000, 0.000000,
            0.000000, 1.000000, 0.000000,
            0.000000, 0.000000, 1.000000
        ]
        info.r = [float(x) for x in info.r]
        
        # Projection Matrix P
        # [fx',  0, cx', Tx,
        #   0, fy', cy', Ty,
        #   0,   0,   1,  0]
        info.p = [
            623.299142,   0.000000, 318.086515, 0.000000,
              0.000000, 628.892192, 267.982220, 0.000000,
              0.000000,   0.000000,   1.000000, 0.000000
        ]
        info.p = [float(x) for x in info.p]
        
        return info

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Timestamp syncing is vital for tf and tracking nodes
            current_time = self.get_clock().now().to_msg()
            
            # 1. Process and Publish Image
            ros_image = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.stamp = current_time
            ros_image.header.frame_id = "camera_frame"
            self.image_pub.publish(ros_image)
            
            # 2. Sync and Publish Camera Info
            self.camera_info_msg.header.stamp = current_time
            self.camera_info_msg.header.frame_id = "camera_frame"
            self.info_pub.publish(self.camera_info_msg)
        else:
            self.get_logger().warn('Failed to grab frame from camera.')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    usb_cam_publisher = UsbCamPublisher()
    
    try:
        rclpy.spin(usb_cam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        usb_cam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
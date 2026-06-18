#!/usr/bin/env python3

import math
import numpy as np

# Fix for old transforms3d with NumPy >= 1.24 / 1.26
if not hasattr(np, "float"):
    np.float = float

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose

from tf_transformations import euler_from_quaternion, quaternion_from_euler


class PathGenerator(Node):

    def __init__(self):
        super().__init__('path_generator')

        self.declare_parameter('entity_name', 'ssmr')
        self.declare_parameter('resolution', 0.05)    # meters
        self.declare_parameter('test_path', 0)

        print(self.get_parameter("test_path").value)
        print(type(self.get_parameter("test_path").value))

        self.client = self.create_client(
            GetEntityState,
            '/gazebo/get_entity_state'
        )

        self.path_pub = self.create_publisher(
            Path,
            '/refined_path',
            10
        )

        self.position_sub = self.create_subscription(
            Pose,
            'ssmr_position_states_for_matlab',
            self.read_pose,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            '/refined_path',
            self.path_callback,
            10
        )

        self.timer_path = self.create_timer(
            1,
            self.timer_path_callback
        )
        # self.timer = self.create_timer(
        #     0.01,
        #     self.timer_callback
        # )
        self.timer2 = self.create_timer(
            0.001,
            self.publish_waypoint
        )
        self.waypoint_pub = self.create_publisher(
            Pose,
            'waypoint',
            10
        )
        self.current_wp_idx = 0
        self.waypoint_reached_thresh = 0.05  # meters

        def precomputed_path_msg(
            frame_id='world',
            x0=0.0,
            y0=0.0,
            theta0=0.0,
            ds=0.1
        ):
            """
            Path shape:
            1) Arc:  R=3m, +90 deg
            2) Straight: 5m
            3) Arc:  R=5m, -180 deg

            Returns:
            nav_msgs.msg.Path
            """

            path_msg = Path()
            path_msg.header.frame_id = frame_id

            x, y, theta = x0, y0, theta0

            def add_pose():
                ps = PoseStamped()
                ps.header.frame_id = frame_id
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.position.z = 0.0

                qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
                ps.pose.orientation.x = qx
                ps.pose.orientation.y = qy
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw

                path_msg.poses.append(ps)

            add_pose()

            # -----------------------
            # 1) Arc: R = 3, +90 deg
            # -----------------------
            R1 = 5.0
            arc1 = math.pi / 2.0
            dtheta = ds / R1
            steps = int(abs(arc1) / dtheta)

            for _ in range(steps):
                theta += dtheta
                x += R1 * (math.sin(theta) - math.sin(theta - dtheta))
                y += -R1 * (math.cos(theta) - math.cos(theta - dtheta))
                add_pose()

            # -----------------------
            # 2) Straight: 5 meters
            # -----------------------
            straight_len = 5.0
            steps = int(straight_len / ds)

            for _ in range(steps):
                x += ds * math.cos(theta)
                y += ds * math.sin(theta)
                add_pose()

            # --------------------------------
            # 3) Arc: R = 5, -180 deg
            # --------------------------------
            R2 = 3.0
            arc2 = math.pi
            dtheta = ds / R2
            steps = int(abs(arc2) / dtheta)

            for _ in range(steps):
                theta -= dtheta
                x += -R2 * (math.sin(theta) - math.sin(theta + dtheta))
                y += R2 * (math.cos(theta) - math.cos(theta + dtheta))
                add_pose()

            return path_msg
        
        if self.get_parameter("test_path").value:
            self.path: Path = precomputed_path_msg(ds=self.get_parameter('resolution').value)
            self.get_logger().info(f'pose len: {len(self.path.poses)}')
        else:
            self.path: Path = None
        self.pose = Pose()
        self.get_logger().info('Path generator node started')

    def timer_path_callback(self):
        if self.get_parameter("test_path").value:
            self.path_pub.publish(self.path)

    def path_callback(self, msg: Path):
        print("asak")
        self.path = msg
        self.get_logger().info("path updated")

    def read_pose(self, msg: Pose):
        self.pose = msg

    def publish_waypoint(self):
        """
        Tracks robot position and publishes the next waypoint along the path.
        """

        if not self.path or not self.path.poses:
            return

        # Current robot position
        rx = self.pose.position.x
        ry = self.pose.position.y

        # -------------------------------
        # 1) Find closest waypoint index
        # -------------------------------
        min_dist = float('inf')
        closest_idx = self.current_wp_idx

        path_lenght = len(self.path.poses)
        for i in range(self.current_wp_idx, self.current_wp_idx + 200):
            idx = min(i, path_lenght-1)
            px = self.path.poses[idx].pose.position.x
            py = self.path.poses[idx].pose.position.y

            d = math.hypot(px - rx, py - ry)
            if d < min_dist:
                min_dist = d
                closest_idx = idx

        self.current_wp_idx = closest_idx

        # --------------------------------
        # 2) Advance if waypoint is reached
        # --------------------------------
        if min_dist < self.waypoint_reached_thresh:
            self.current_wp_idx += 1

        # Clamp index
        if self.current_wp_idx >= len(self.path.poses):
            self.current_wp_idx = len(self.path.poses) - 1

        # -------------------------
        # 3) Publish waypoint
        # -----------------------
        wp: Pose = self.path.poses[self.current_wp_idx].pose
        # wp.position.x -= 4
        # wp.position.y -= 6

        # wp.orientation.w = float(self.current_wp_idx)

        self.waypoint_pub.publish(wp)
        # self.get_logger().info(f"\npose len: {len(self.path.poses)}\nwp num: {self.current_wp_idx}\nwp x: {wp.position.x}\nwp y: {wp.position.y}\nwp z: {wp.position.z}\nwp: {wp.orientation.x}\nwp: {wp.orientation.y}\nwp: {wp.orientation.z}\nwp: {wp.orientation.w}\npose:\nx: {self.pose.position.x}\ny: {self.pose.position.y}\n")
        # self.get_logger().info(f"\n")


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

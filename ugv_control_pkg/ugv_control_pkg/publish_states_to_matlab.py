#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import EntityState

from geometry_msgs.msg import Twist, Quaternion, Pose
from std_msgs.msg import Float64MultiArray

import numpy as np, math

def quat_to_rot_matrix(q: Quaternion):
    x, y, z, w = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])
    return R

class MyNode(Node):

    def __init__(self):
        super().__init__('publish_states_to_matlab')

        # -------- Gazebo state client --------
        self.get_state_cli = self.create_client(
            GetEntityState, 
            '/gazebo/get_entity_state'
        )
        self.timer_ = self.create_timer(
            0.01,
            self.timer_callback_
        )
        self.vel_state_pub_ = self.create_publisher(
            Twist,
            'ssmr_velocity_states_for_matlab',
            10
        )
        self.pose_state_pub_ = self.create_publisher(
            Pose,
            'ssmr_position_states_for_matlab',
            10
        )

        entity_name = 'ssmr'
        reference_frame = 'world'
        self.get_req = GetEntityState.Request()
        self.get_req.name = entity_name
        self.get_req.reference_frame = reference_frame

        # -------- Command effort subscriber --------
        self.command_sub_ = self.create_subscription(
            Twist,
            '/command_effort',
            self.command_effort_callback,
            10
        )

        # -------- Wheel effort publisher --------
        self.wheel_effort_pub_ = self.create_publisher(
            Float64MultiArray,
            '/wheel_effort_cont/commands',
            10
        )

        self.get_state_cli.wait_for_service()
        self.get_logger().info('publish_states_to_matlab node initialized')

    # ================= Gazebo state =================
    def timer_callback_(self):
        future = self.get_state_cli.call_async(self.get_req)
        future.add_done_callback(self.get_state_done_callback)

    def get_state_done_callback(self, future):
        current_state: EntityState = future.result().state
        velocities: Twist = current_state.twist

        # world-frame velocity
        v_world = np.array([
            velocities.linear.x,
            velocities.linear.y,
            velocities.linear.z
        ])
        # rotation matrix
        R = quat_to_rot_matrix(current_state.pose.orientation)
        # transform to robot frame
        v_robot = R.T @ v_world
        # signed forward speed
        velocities.linear.x = v_robot[0] if abs(v_robot[0]) > 0.05 else 0.0
        velocities.linear.y = v_robot[1] if abs(v_robot[1]) > 0.05 else 0.0
        velocities.linear.z = v_robot[2] if abs(v_robot[2]) > 0.05 else 0.0
        
        velocities.angular.z = velocities.angular.z if abs(velocities.angular.z) > 0.05 else 0.0

        self.vel_state_pub_.publish(velocities)

        position = current_state.pose
        self.pose_state_pub_.publish(position)
        # self.get_logger().info(f"\nx: {position.position.x}\ny: {position.position.y}\nz: {position.position.z}")
        

    # ================= Command effort =================
    def command_effort_callback(self, msg: Twist):
        """
        Reads:
          linear.x, linear.y, linear.z
          angular.x
        Publishes to /wheel_effort_cont/commands
        """
        def clean_nan(val):
            return 0.0 if math.isnan(val) else val

        effort_msg = Float64MultiArray()

        # Your updated assignment
        effort_msg.data = [
            clean_nan(msg.linear.x),
            clean_nan(msg.linear.y),
            clean_nan(msg.linear.z),
            clean_nan(msg.angular.x)
        ]

        self.wheel_effort_pub_.publish(effort_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

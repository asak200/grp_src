#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# class Camera_position_request_sender(Node):

#     def __init__(self):
#         super().__init__('camera_position_request_sender')


# def main(args=None):
#     rclpy.init(args=args)
#     node = Camera_position_request_sender()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node

import cv2

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from functools import partial


class CameraPositionController(Node):

    def __init__(self):
        super().__init__('camera_position_controller')

        self.entity_name = 'free_camera'
        self.reference_frame = 'world'

        # Subscriber
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Service clients
        self.get_state_cli = self.create_client(
            GetEntityState,
            '/gazebo/get_entity_state'
        )

        self.set_state_cli = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        self.get_logger().info('Waiting for Gazebo services...')
        self.get_state_cli.wait_for_service()
        self.set_state_cli.wait_for_service()
        self.get_logger().info('asak Camera position controller ready.')

    def cmd_callback(self, msg: Twist):
        self.get_logger().info('msg recieved')
        # 1) Get current state
        get_req = GetEntityState.Request()
        get_req.name = self.entity_name
        get_req.reference_frame = self.reference_frame

        future = self.get_state_cli.call_async(get_req)
        future.add_done_callback(partial(self.get_state_done_callback, msg=msg))
    
    def get_state_done_callback(self, future, msg: Twist):
        self.get_logger().info('getting state')

        current_state: EntityState = future.result().state

        # 2) Extract current pose
        pos = current_state.pose.position
        ori = current_state.pose.orientation

        roll, pitch, yaw = euler_from_quaternion([
            ori.x, ori.y, ori.z, ori.w
        ])

        # 3) Apply increments
        pos.x = msg.linear.x
        pos.y = msg.linear.y
        pos.z = 0.06
        yaw   = 0.0
        pitch = 0.0
        roll = 0.0

        self.get_logger().info(f'state: {pos.x}, {pos.y}, {pos.z}')
        q = quaternion_from_euler(roll, pitch, yaw)
          
        # 4) Build new state
        new_state = EntityState()
        new_state.name = self.entity_name
        new_state.reference_frame = self.reference_frame

        new_state.pose.position = pos
        new_state.pose.orientation.x = q[0]
        new_state.pose.orientation.y = q[1]
        new_state.pose.orientation.z = q[2]
        new_state.pose.orientation.w = q[3]

        # Keep velocity zero
        new_state.twist.linear.x = 0.0
        new_state.twist.linear.y = 0.0
        new_state.twist.linear.z = 0.0
        new_state.twist.angular.x = 0.0
        new_state.twist.angular.y = 0.0
        new_state.twist.angular.z = 0.0

        # 5) Send update
        set_req = SetEntityState.Request()
        set_req.state = new_state

        set_future = self.set_state_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, set_future)

        if not set_future.result():
            self.get_logger().error('Failed to set entity state')


def main():
    rclpy.init()
    node = CameraPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
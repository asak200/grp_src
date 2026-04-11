#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

class ApplyTorqueFromVoltage(Node):

    def __init__(self):
        super().__init__('apply_torque_from_voltage')
        self.voltage_listener_ = self.create_subscription(
            Vector3,
            '/ssmr/voltages',
            self.voltage_listener_callback,
            10
        )
        self.joint_states_listener_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.apply_torque_timer_ = self.create_timer(0.02, self.apply_torque_)
        self.apply_torque_pub_ = self.create_publisher(
            Float64MultiArray,
            '/wheel_effort_cont/commands',
            10
        )
        
        self.gear_ratio = 17
        self.internal_resistance = 7.8
        self.emf_constant = 0.012
        self.torque_constant = 0.012
        self.voltages = [0, 0]
        self.armuture_currents = [0, 0]
        self.joint_wheel_velocities = [0, 0, 0, 0]

        self.get_logger().info('apply_torque_from_voltage node initialized')

    def joint_states_callback(self, msg: JointState):
        self.joint_wheel_velocities = msg.velocity
        # self.get_logger().info(f'vs: {self.joint_wheel_velocities[0]}')
    
    def voltage_listener_callback(self, msg: Vector3):
        self.voltages = [msg.x, msg.y]
        # self.get_logger().info(f'voltages: {self.voltages}')

    def apply_torque_(self):
        # voltage = internal_resistance * armuture_current + gear_ratio * emf_constant * joint_wheel_velocity
        # armuture_current = (voltage - gear_ratio * emf_constant * joint_wheel_velocity) / internal_resistance
        # torque = gear_ratio * torque_constant * armuture_current
        msg = Float64MultiArray()
        calculated_torques = []
        
        for i in range(2):
            # Calculate Back-EMF (using velocity at the motor shaft)
            v_back_emf = self.gear_ratio * self.emf_constant * self.joint_wheel_velocities[i]
            
            # Calculate Current
            current = (self.voltages[i] - v_back_emf) / self.internal_resistance
            
            # Calculate Torque at the motor shaft, then apply gear ratio for output torque
            # Note: Torque at wheel = Motor Torque * Gear Ratio
            torque_wheel = self.gear_ratio * self.torque_constant * current
            calculated_torques.append(torque_wheel)
            calculated_torques.append(torque_wheel)
            
        msg.data = calculated_torques
        # self.get_logger().info(f"torques: {msg.data}")
        self.apply_torque_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ApplyTorqueFromVoltage()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

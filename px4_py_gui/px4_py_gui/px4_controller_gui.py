#!/usr/bin/env python3

import rclpy, time
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleOdometry, VehicleCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy

from scipy.spatial.transform import Rotation as R
import csv
from tkinter import *

PINK = "#e2979c"
RED = "#e7305b"
GREEN = "#00aa00"
BLUE = "#008eff"
YELLOW = "#f7f5dd"
WHITE =  "#ffffff"
FONT_NAME = "Times New Roman"

class MyNode(Node):

    def __init__(self):
        super().__init__('px4_controller_gui')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.declare_parameter('target_system_id', 1)
        self.target_system_id = self.get_parameter('target_system_id').value

        # /px4_{self.target_system_id-1}
        self.pub_ = self.create_publisher(TrajectorySetpoint, 'set_point_'+str(self.target_system_id-1), 10)
        self.type_trigger_ = self.create_publisher(OffboardControlMode, 'control_type_'+str(self.target_system_id-1), 10)
        
        # for single drone sim
        self.odom_listener_ = self.create_subscription(VehicleOdometry, f'/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        # for multidrone sim
        # self.odom_listener_ = self.create_subscription(VehicleOdometry, f'/px4_{self.target_system_id-1}/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.start_time = time.time()
        
        self.x_ref: float = 0.
        self.y_ref: float = -1.
        self.z_ref: float = -10.
        self.yaw_ref: float = 90.
        self.vx_ref: float = 0.
        self.vy_ref: float = 0.
        self.vz_ref: float = 0.
        self.yawspeed_ref: float = 0.

        self.x_act: float = 0.
        self.y_act: float = 0.
        self.z_act: float = 7.
        self.yaw_act: float = 90.
        self.vx_act: float = 0.
        self.vy_act: float = 0.
        self.vz_act: float = 0.
        self.yawspeed_act: float = 0.
        
        self.window = Tk()
        self.make_main_window()

        self.window_update_timer_ = self.create_timer(0.05, self.window.update)

        # self.log_file = open('~/dasal/src/log.csv', mode='w', newline='')
        # self.csv_writer = csv.writer(self.log_file)

        # self.csv_writer.writerow(['time_s', 'x_ref', 'x_act', 'vx_ref', 'vx_act', 'y_ref', 'y_act', 'vy_ref', 'vy_act', 
        #                           'z_ref', 'z_act', 'vz_ref', 'vz_act', 'yaw_ref', 'yaw_act', 'yawspeed_ref', 'yawspeed_act'])
        
        self.get_logger().info("px4_controller_gui initilized")
        
    def make_main_window(self):
        self.window.config(padx=50, pady=10, bg=YELLOW)
        welcom_label = Label(self.window, text=f'px4_sitl Control Panel\nFor Drone {self.target_system_id-1}', bg=YELLOW, fg=GREEN,
                            highlightthickness=0, font=(FONT_NAME, 20, 'bold'),
                            padx=50, pady=10)
        welcom_label.grid(column=0, row=0, columnspan=3)

        x_lab = Label(self.window, text='x: ', bg=YELLOW, fg=GREEN, highlightthickness=0, font=(FONT_NAME, 15, 'bold'), padx=0, pady=10)
        x_lab.grid(column=0, row=1)
        y_lab = Label(self.window, text='y: ', bg=YELLOW, fg=GREEN, highlightthickness=0, font=(FONT_NAME, 15, 'bold'), padx=0, pady=10)
        y_lab.grid(column=0, row=2)
        z_lab = Label(self.window, text='z: ', bg=YELLOW, fg=GREEN, highlightthickness=0, font=(FONT_NAME, 15, 'bold'), padx=0, pady=10)
        z_lab.grid(column=0, row=3)
        yaw_lab = Label(self.window, text='yaw: ', bg=YELLOW, fg=GREEN, highlightthickness=0, font=(FONT_NAME, 15, 'bold'), padx=0, pady=10)
        yaw_lab.grid(column=0, row=4)

        self.x_in = Entry(self.window, font=(FONT_NAME, 15), bg="#FFFFFF", fg="#000000")
        self.x_in.grid(column=1, row=1)
        self.y_in = Entry(self.window, font=(FONT_NAME, 15), bg="#FFFFFF", fg="#000000")
        self.y_in.grid(column=1, row=2)
        self.z_in = Entry(self.window, font=(FONT_NAME, 15), bg="#FFFFFF", fg="#000000")
        self.z_in.grid(column=1, row=3)
        self.yaw_in = Entry(self.window, font=(FONT_NAME, 15), bg="#FFFFFF", fg="#000000")
        self.yaw_in.grid(column=1, row=4)

        send_waypoint_button = Button(self.window, text="Send The \nWay Point", bg=GREEN, width=15, 
                                    command=self.send_way_point, padx=0, pady=0, 
                                    font=(FONT_NAME, 20, 'bold'))
        send_waypoint_button.grid(column=2, row=1, rowspan=2)
        send_velocity_button = Button(self.window, text="Send The \nVelocity", bg=GREEN, width=15, 
                                    command=self.send_velocity, padx=0, pady=0, 
                                    font=(FONT_NAME, 20, 'bold'))
        send_velocity_button.grid(column=2, row=3, rowspan=2)

        self.x_lab = Label(self.window, text=f'Velocities', bg=YELLOW, fg=GREEN, highlightthickness=0, font=(FONT_NAME, 15, 'bold'), padx=0, pady=10)
        self.x_lab.grid(column=0, row=5, columnspan=3)

        self.x_lab = Label(self.window, text=f'{self.vx_act}', bg=YELLOW, fg=GREEN, highlightthickness=0, font=(FONT_NAME, 15, 'bold'), padx=0, pady=10)
        self.x_lab.grid(column=0, row=6)
        self.y_lab = Label(self.window, text=f'{self.vy_act}', bg=YELLOW, fg=GREEN, highlightthickness=0, font=(FONT_NAME, 15, 'bold'), padx=0, pady=10)
        self.y_lab.grid(column=1, row=6)
        self.z_lab = Label(self.window, text=f'{self.vz_act}', bg=YELLOW, fg=GREEN, highlightthickness=0, font=(FONT_NAME, 15, 'bold'), padx=0, pady=10)
        self.z_lab.grid(column=2, row=6)

        # gimbal_button = Button(self.window, text="Send Gimbal\n Command", bg=GREEN,
        #                             command=self.send_gimbal, padx=0, pady=0,
        #                             font=(FONT_NAME, 20, 'bold'))
        # gimbal_button.grid(column=3, row=1)

    def send_way_point(self):
        control_type_msg = OffboardControlMode()
        control_type_msg.position = True
        control_type_msg.velocity = False
        self.type_trigger_.publish(control_type_msg)

        self.x_ref = float(self.x_in.get())
        self.y_ref = float(self.y_in.get())
        self.z_ref = float(self.z_in.get())
        self.yaw_ref = float(self.yaw_in.get())

        msg = TrajectorySetpoint()
        msg.timestamp = 0 # ignore time
        msg.position[0] = self.x_ref
        msg.position[1] = self.y_ref
        msg.position[2] = self.z_ref
        msg.yaw = self.yaw_ref * 3.14159 / 180
        msg.yawspeed = 0.0

        self.pub_.publish(msg)
        self.get_logger().info(f"sent a waypoint to x: {self.x_ref}, y: {self.y_ref}, z: {self.x_ref}, yaw: {self.yaw_ref}")

        if self.start_time is None:
            self.start_time = time.time()

    def send_velocity(self):
        control_type_msg = OffboardControlMode()
        control_type_msg.position = False
        control_type_msg.velocity = True
        self.type_trigger_.publish(control_type_msg)

        self.vx_ref = float(self.x_in.get())
        self.vy_ref = float(self.y_in.get())
        self.vz_ref = float(self.z_in.get())
        self.yawspeed_ref = float(self.yaw_in.get())

        msg = TrajectorySetpoint()
        msg.timestamp = 0 # ignore time
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        msg.velocity[0] = self.vx_ref
        msg.velocity[1] = self.vy_ref
        msg.velocity[2] = self.vz_ref
        msg.yaw = float('nan')
        msg.yawspeed = self.yawspeed_ref * 3.14159 / 180

        self.pub_.publish(msg)
        self.get_logger().info(f"sent a velocity command x: {self.vx_ref}, y: {self.vy_ref}, z: {self.vz_ref}, yawspead: {self.yawspeed_ref}")

    def get_yaw_deg_from_px4_quat(self, q: VehicleOdometry.q)-> float | int:
        quat_scipy = [q[1], q[2], q[3], q[0]]  # Convert PX4 [w,x,y,z] -> [x,y,z,w]
        r = R.from_quat(quat_scipy)
        return r.as_euler('xyz', degrees=True)[2]  # Yaw

    def odom_callback(self, msg: VehicleOdometry):
        self.x_act = round(msg.position[0]*100)/100.
        self.y_act = round(msg.position[1]*100)/100.
        self.z_act = round(msg.position[2]*100)/100.


        self.yaw_act = self.get_yaw_deg_from_px4_quat(msg.q)
        # print(self.yaw_act, self.yaw_ref)
        # self.get_logger().info(f"{self.yaw_act, self.yaw_ref}")

        self.vx_act = round(msg.velocity[0]*100)/100.
        self.vy_act = round(msg.velocity[1]*100)/100.
        self.vz_act = round(msg.velocity[2]*100)/100.
        self.x_lab.config(text=f'{self.vx_act}')
        self.y_lab.config(text=f'{self.vy_act}')
        self.z_lab.config(text=f'{self.vz_act}')

        
        # current_time = time.time() - self.start_time

        # self.csv_writer.writerow([current_time,
        #                           self.x_ref,
        #                           self.x_act,
        #                           self.vx_ref,
        #                           self.vx_act,
        #                           self.y_ref,
        #                           self.y_act,
        #                           self.vy_ref,
        #                           self.vy_act,
        #                           self.z_ref,
        #                           self.z_act,
        #                           self.vz_ref,
        #                           self.vz_act,
        #                           self.yaw_ref,
        #                           self.yaw_act,
        #                           self.yawspeed_ref,
        #                           self.yawspeed_act,
        #                         ])
        # self.log_file.flush()

    # def send_gimbal(self):
    #     msg = VehicleCommand()
    #     msg.timestamp = int(time.time() * 1e6)
    #     msg.param1 = 30.0   # pitch (down = negative)
    #     msg.param2 = 80.0     # roll
    #     msg.param3 = 0.0     # yaw
    #     msg.param4 = float('nan')
    #     msg.param5 = float('nan')
    #     msg.param6 = float('nan')
    #     msg.param7 = float('nan')

    #     msg.command = 205  # MAV_CMD_DO_MOUNT_CONTROL
    #     msg.target_system = self.target_system_id
    #     msg.target_component = 1
    #     msg.source_system = 1
    #     msg.source_component = 1
    #     msg.confirmation = 0
    #     msg.from_external = True

    #     self.gimbal_pub_.publish(msg)
    #     self.get_logger().info('Sent gimbal control command.')


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    
    node.log_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

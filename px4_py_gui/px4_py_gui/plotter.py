#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import sys
import pandas as pd
import matplotlib.pyplot as plt

# TODO: 
#   add parameters:
#       log file path
#       list of plotting axixes

class PlotLogNode(Node):
    def __init__(self):
        super().__init__('plot_log_node')
        if "--pose" in sys.argv:
            i = sys.argv.index('--pose')
            mode = sys.argv[i+1]
            if mode == '0':
                self.plot_vel_csv_data()
            elif mode == '1':
                self.plot_pos_csv_data()
        else:
            self.get_logger().fatal("argument --pose is not set!")

    def plot_vel_csv_data(self):
        try:
            df = pd.read_csv('~/dasal/src/log.csv')
        except FileNotFoundError:
            self.get_logger().error("log.csv not found!")
            return
        
        time_s = df['time_s'].to_numpy()
        vx_ref = df['vx_ref'].to_numpy()
        vx_act = df['vx_act'].to_numpy()
        vy_ref = df['vy_ref'].to_numpy()
        vy_act = df['vy_act'].to_numpy()
        vz_ref = df['vz_ref'].to_numpy()
        vz_act = df['vz_act'].to_numpy()

        plt.figure(figsize=(7, 5))
        plt.plot(time_s, vx_ref, label='vx_ref')
        plt.plot(time_s, vx_act, label='vx_act')
        plt.xlabel("Time (s)")
        plt.ylabel("Position X")
        plt.title("X Tracking Velocity Performance")
        plt.legend()
        plt.grid(True)
        plt.show(block=False)

        plt.figure(figsize=(7, 5))
        plt.plot(time_s, vy_ref, label='vy_ref')
        plt.plot(time_s, vy_act, label='vy_act')
        plt.xlabel("Time (s)")
        plt.ylabel("Position Y")
        plt.title("Y Tracking Velocity Performance")
        plt.legend()
        plt.grid(True)
        plt.show(block=False)

        plt.figure(figsize=(7, 5))
        plt.plot(time_s, vz_ref, label='vz_ref')
        plt.plot(time_s, vz_act, label='vz_act')
        plt.xlabel("Time (s)")
        plt.ylabel("Position Z")
        plt.title("Z Tracking Velocity Performance")
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_pos_csv_data(self):
        try:
            df = pd.read_csv('~/dasal/src/log.csv')
        except FileNotFoundError:
            self.get_logger().error("log.csv not found!")
            return
        
        time_s = df['time_s'].to_numpy()
        x_ref = df['x_ref'].to_numpy()
        x_act = df['x_act'].to_numpy()
        y_ref = df['y_ref'].to_numpy()
        y_act = df['y_act'].to_numpy()
        z_ref = df['z_ref'].to_numpy()
        z_act = df['z_act'].to_numpy()

        plt.figure(figsize=(7, 5))
        plt.plot(time_s, x_ref, label='x_ref')
        plt.plot(time_s, x_act, label='x_act')
        plt.xlabel("Time (s)")
        plt.ylabel("Position X")
        plt.title("X Tracking Performance")
        plt.legend()
        plt.grid(True)
        plt.show(block=False)

        plt.figure(figsize=(7, 5))
        plt.plot(time_s, y_ref, label='y_ref')
        plt.plot(time_s, y_act, label='y_act')
        plt.xlabel("Time (s)")
        plt.ylabel("Position Y")
        plt.title("Y Tracking Performance")
        plt.legend()
        plt.grid(True)
        plt.show(block=False)

        plt.figure(figsize=(7, 5))
        plt.plot(time_s, z_ref, label='z_ref')
        plt.plot(time_s, z_act, label='z_act')
        plt.xlabel("Time (s)")
        plt.ylabel("Position Z")
        plt.title("Z Tracking Performance")
        plt.legend()
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PlotLogNode()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

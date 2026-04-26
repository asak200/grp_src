# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from nav_msgs.msg import Path


# class MyNode(Node):

#     def __init__(self):
#         super().__init__('choose_path')

#         self.image_sub = self.create_subscription(Path, '/ugv/planned_path', self.path_callback, 10)
#         self.timer_ = self.create_timer(1.0, self.timer_callback)
#         self.publish_path_ = self.create_publisher(Path, "refined_path", 10)

#         self.get_logger().info("choose_path node initialized")

#     def timer_callback(self):
#         self.publish_path_.publish(self.latest_path)

#     def path_callback(self, msg: Path):
#         self.latest_path = msg
#         print(len(msg.poses))


# def main(args=None):
#     rclpy.init(args=args)
#     node = MyNode()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path


class MyNode(Node):

    def __init__(self):
        super().__init__('choose_path')

        # Subscribe to incoming planned paths
        self.path_sub = self.create_subscription(
            Path,
            '/ugv/planned_path',
            self.path_callback,
            10
        )

        # Publisher for the best (shortest) path
        self.publish_path_ = self.create_publisher(
            Path,
            'refined_path',
            10
        )

        # Timer to continuously publish the shortest path found
        self.timer_ = self.create_timer(
            10.0,
            self.timer_callback
        )

        self.shortest_path = None
        self.shortest_distance = float('inf')

        self.get_logger().info("choose_path node initialized")

    def calculate_path_distance(self, path_msg: Path):
        """
        Calculate total Euclidean distance of the path
        by summing distances between consecutive poses.
        """
        if len(path_msg.poses) < 2:
            return 0.0

        total_distance = 0.0

        for i in range(1, len(path_msg.poses)):
            prev_pose = path_msg.poses[i - 1].pose.position
            curr_pose = path_msg.poses[i].pose.position

            dx = curr_pose.x - prev_pose.x
            dy = curr_pose.y - prev_pose.y
            dz = curr_pose.z - prev_pose.z

            segment_distance = math.sqrt(dx**2 + dy**2 + dz**2)
            total_distance += segment_distance

        return total_distance

    def path_callback(self, msg: Path):
        current_distance = self.calculate_path_distance(msg)

        # self.get_logger().info(
        #     f"Received path with total distance: {current_distance:.3f} m"
        # )

        # Keep only the shortest path
        if current_distance < self.shortest_distance:
            self.shortest_distance = current_distance
            self.shortest_path = msg

            self.get_logger().info(
                f"New shortest path stored: {self.shortest_distance:.3f} m"
            )

    def timer_callback(self):
        if self.shortest_path is not None:
            self.publish_path_.publish(self.shortest_path)

            self.get_logger().info(
                f"Publishing shortest path: {self.shortest_distance:.3f} m"
            )
            self.shortest_distance = float('inf')
            # self.timer_.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
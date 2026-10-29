"""Record a waypoint CSV by driving a lap (for the F1TENTH track; CARLA uses the exported lane graph).

ros2 run planning_sim waypoint_recorder --ros-args -p output:=levine.csv
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("waypoint_recorder")
        self.declare_parameter("output", "waypoints.csv")
        self.declare_parameter("pose_topic", "/ground_truth/odom")
        self.declare_parameter("spacing", 0.2)
        self.file = open(self.get_parameter("output").value, "w")
        self.file.write("# x, y\n")
        self.last = None
        self.count = 0
        self.create_subscription(Odometry, self.get_parameter("pose_topic").value, self.on_pose, 50)

    def on_pose(self, msg):
        p = msg.pose.pose.position
        if (
            self.last is None
            or math.hypot(p.x - self.last[0], p.y - self.last[1]) >= self.get_parameter("spacing").value
        ):
            self.file.write(f"{p.x:.3f}, {p.y:.3f}\n")
            self.file.flush()
            self.last = (p.x, p.y)
            self.count += 1


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        print(f"saved {node.count} waypoints")
        node.destroy_node()
        rclpy.try_shutdown()

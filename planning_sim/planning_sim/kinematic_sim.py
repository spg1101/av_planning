"""Tiny kinematic bicycle simulator: the fastest closed-loop test, no gym, no GPU.

In:  /drive (AckermannDriveStamped), /planning/trajectory (only in perfect_tracking mode)
Out: TF map -> base_link, /ground_truth/odom, /vehicle/odom

perfect_tracking=true snaps the car onto the latest trajectory (the "perfect tracker"
stand-in for controls): planning logic can be tested before any controller is tuned.
Use without localization: this node publishes map -> base_link directly.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from av_msgs.msg import Trajectory as TrajectoryMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from planning_core.vehicle import VehicleParams
from planning_nodes.conversions import msg_to_trajectory, yaw_to_quaternion


class KinematicSim(Node):
    def __init__(self):
        super().__init__("kinematic_sim")
        self.declare_parameter("vehicle_file", "")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("perfect_tracking", False)
        self.vehicle = VehicleParams.from_yaml(self.get_parameter("vehicle_file").value)
        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)
        self.v = 0.0
        self.cmd_speed, self.cmd_steer = 0.0, 0.0
        self.traj, self.traj_rx_time = None, None
        self.dt = 1.0 / self.get_parameter("rate_hz").value

        self.tf = TransformBroadcaster(self)
        self.gt_pub = self.create_publisher(Odometry, "/ground_truth/odom", 10)
        self.odom_pub = self.create_publisher(Odometry, "/vehicle/odom", 10)
        self.create_subscription(AckermannDriveStamped, "/drive", self.on_drive, 10)
        self.create_subscription(TrajectoryMsg, "/planning/trajectory", self.on_traj, 10)
        self.create_timer(self.dt, self.step)

    def on_drive(self, msg):
        self.cmd_speed = msg.drive.speed
        self.cmd_steer = max(-self.vehicle.max_steer, min(self.vehicle.max_steer, msg.drive.steering_angle))

    def on_traj(self, msg):
        self.traj = msg_to_trajectory(msg)
        self.traj_rx_time = self.get_clock().now().nanoseconds * 1e-9

    def step(self):
        if self.get_parameter("perfect_tracking").value and self.traj and self.traj.points:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.traj_rx_time
            p = min(self.traj.points, key=lambda q: abs(q.time_from_start - elapsed))
            self.x, self.y, self.yaw, self.v = p.x, p.y, p.yaw, p.velocity
            omega = p.velocity * p.curvature
        else:
            dv = self.cmd_speed - self.v
            limit = (self.vehicle.max_accel if dv > 0 else self.vehicle.max_decel) * self.dt
            self.v += max(-limit, min(limit, dv))
            omega = self.v / self.vehicle.wheelbase * math.tan(self.cmd_steer)
            self.x += self.v * math.cos(self.yaw) * self.dt
            self.y += self.v * math.sin(self.yaw) * self.dt
            self.yaw = math.atan2(math.sin(self.yaw + omega * self.dt), math.cos(self.yaw + omega * self.dt))
        self.publish(omega)

    def publish(self, omega):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, "map", "base_link"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation = yaw_to_quaternion(self.yaw)
        self.tf.sendTransform(t)

        gt = Odometry()
        gt.header.stamp, gt.header.frame_id, gt.child_frame_id = now, "map", "base_link"
        gt.pose.pose.position.x, gt.pose.pose.position.y = self.x, self.y
        gt.pose.pose.orientation = t.transform.rotation
        gt.twist.twist.linear.x, gt.twist.twist.angular.z = self.v, omega
        self.gt_pub.publish(gt)

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, "odom", "base_link"
        odom.twist = gt.twist
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = KinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

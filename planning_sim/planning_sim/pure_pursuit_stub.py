"""Pure pursuit + speed passthrough: the controls stand-in until the controls team's node.

In:  /planning/trajectory, TF map->base_link (pose), velocity topic (speed)
Out: /drive (AckermannDriveStamped) at 50 Hz
Safety: no trajectory for `timeout` seconds or emergency_stop -> commands zero speed.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from av_msgs.msg import Trajectory as TrajectoryMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from planning_core.vehicle import VehicleParams
from planning_nodes.conversions import msg_to_trajectory, quaternion_to_yaw


class PurePursuitStub(Node):
    def __init__(self):
        super().__init__("pure_pursuit_stub")
        self.declare_parameter("vehicle_file", "")
        self.declare_parameter("velocity_topic", "/ground_truth/odom")
        self.declare_parameter("lookahead_min", 0.6)
        self.declare_parameter("lookahead_gain", 0.3)
        self.declare_parameter("timeout", 0.3)
        self.declare_parameter("rate_hz", 50.0)
        self.vehicle = VehicleParams.from_yaml(self.get_parameter("vehicle_file").value)
        self.traj, self.rx_time, self.speed = None, None, 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.create_subscription(TrajectoryMsg, "/planning/trajectory", self.on_traj, 10)
        self.create_subscription(
            Odometry,
            self.get_parameter("velocity_topic").value,
            lambda m: setattr(self, "speed", m.twist.twist.linear.x),
            10,
        )
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def on_traj(self, msg):
        self.traj = msg_to_trajectory(msg)
        self.rx_time = self.get_clock().now()

    def send(self, speed, steer):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steer)
        self.pub.publish(cmd)

    def tick(self):
        timeout = self.get_parameter("timeout").value
        stale = self.rx_time is None or (self.get_clock().now() - self.rx_time).nanoseconds * 1e-9 > timeout
        if stale or self.traj.emergency_stop or not self.traj.points:
            self.send(0.0, 0.0)
            return
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            self.send(0.0, 0.0)
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        yaw = quaternion_to_yaw(tf.transform.rotation)
        pts = self.traj.points
        i_near = min(range(len(pts)), key=lambda i: math.hypot(pts[i].x - x, pts[i].y - y))
        lookahead = max(
            self.get_parameter("lookahead_min").value,
            self.get_parameter("lookahead_gain").value * abs(self.speed),
        )
        target = pts[-1]
        for p in pts[i_near:]:
            if math.hypot(p.x - x, p.y - y) >= lookahead:
                target = p
                break
        dx, dy = target.x - x, target.y - y
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        dist = max(math.hypot(dx, dy), 1e-3)
        steer = math.atan2(2.0 * self.vehicle.wheelbase * local_y, dist * dist)
        steer = max(-self.vehicle.max_steer, min(self.vehicle.max_steer, steer))
        # Speed from the lookahead point: the plan's first point carries the car's current
        # speed (0 at standstill), so commanding it would never start the car.
        self.send(target.velocity, steer)


def main():
    rclpy.init()
    node = PurePursuitStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

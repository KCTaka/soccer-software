#!/usr/bin/env python3
"""Keyframe trajectory player. Publishes interpolated JointState targets.

Non-real-time. Runs on the executor thread. The controller's subscription
callback writes into the SPSC buffer; the 200 Hz update() reads from it.
"""
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


class TrajectoryPlayer(Node):
    def __init__(self, keyframes: list[dict], joint_names: list[str],
                 dt: float = 0.005, hold_final: float = 5.0):
        super().__init__('trajectory_player')
        self.joint_names = joint_names
        self.keyframes = keyframes
        self.dt = dt
        self.hold_final = hold_final
        self.pub = self.create_publisher(
            JointState,
            '/humanoid_mit_controller/joint_references',
            rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(dt, self._tick)
        self._t = 0.0
        self._done = False

    def _interpolate(self, t: float) -> list[float]:
        """Linear interpolation between keyframes."""
        kf = self.keyframes
        if t <= kf[0]['time']:
            return kf[0]['positions']
        if t >= kf[-1]['time']:
            return kf[-1]['positions']
        for i in range(len(kf) - 1):
            t0, t1 = kf[i]['time'], kf[i + 1]['time']
            if t0 <= t < t1:
                alpha = (t - t0) / (t1 - t0)
                p0, p1 = kf[i]['positions'], kf[i + 1]['positions']
                return [a + alpha * (b - a) for a, b in zip(p0, p1)]
        return kf[-1]['positions']

    def _tick(self):
        if self._done:
            return
        positions = self._interpolate(self._t)
        msg = JointState()
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)
        self.pub.publish(msg)
        self._t += self.dt
        if self._t > self.keyframes[-1]['time'] + self.hold_final:
            self._done = True
            self.get_logger().info('Trajectory complete. Holding final pose.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--keyframes', required=True, help='Path to keyframe YAML')
    parser.add_argument('--hold', type=float, default=10.0,
                        help='Seconds to hold final pose')
    args, _ = parser.parse_known_args()

    with open(args.keyframes) as f:
        data = yaml.safe_load(f)

    rclpy.init()
    node = TrajectoryPlayer(
        keyframes=data['keyframes'],
        joint_names=data['joint_names'],
        dt=data.get('dt', 0.005),
        hold_final=args.hold)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
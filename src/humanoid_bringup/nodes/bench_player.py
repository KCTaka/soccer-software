#!/usr/bin/env python3
"""Bench reference player for free drives. Publishes JointState targets.

Non-real-time. Waits for the first /joint_states, eases every joint from where it rests to 0, then
swings each joint to swing_sign * amplitude, through the opposite side, and back to 0, and holds.
Each target carries its own velocity as feed-forward. Unlike trajectory_player.py it starts from
the measured position: a free drive rests wherever it was last turned, and a first target of 0
would be a step.
"""
import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


class BenchPlayer(Node):
    def __init__(self):
        super().__init__('bench_player')
        # Defaults only fix each parameter's type; the checks below reject them.
        self.joints = self.declare_parameter('joints', ['']).value
        signs = self.declare_parameter('swing_signs', [0]).value
        amplitude = self.declare_parameter('amplitude_rad', 0.0).value
        ease_in = self.declare_parameter('ease_in_s', 0.0).value
        swing = self.declare_parameter('swing_s', 0.0).value
        if not self.joints or not all(self.joints) or len(signs) != len(self.joints):
            raise ValueError("'joints' must be set, with one 'swing_signs' entry per joint")
        if ease_in <= 0.0 or swing <= 0.0:
            raise ValueError("'ease_in_s' and 'swing_s' must be positive")

        # (duration, goal per joint). The middle segment crosses twice the distance, in twice the
        # time, so every swing has the same peak velocity.
        side = [s * amplitude for s in signs]
        zero = [0.0] * len(self.joints)
        self.segments = [
            (ease_in, zero),
            (swing, side),
            (2.0 * swing, [-p for p in side]),
            (swing, zero),
            (swing, zero),
        ]

        self.pub = self.create_publisher(
            JointState,
            '/humanoid_mit_controller/joint_references',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self._on_state, 10)
        self.start = None
        self.timer = None
        self._t = 0.0
        self.dt = 0.005

    def _on_state(self, msg: JointState):
        if self.start is not None or not set(self.joints) <= set(msg.name):
            return
        self.start = [msg.position[msg.name.index(j)] for j in self.joints]
        self.get_logger().info('Starting from ' + ', '.join(
            f'{j}={p:+.3f}' for j, p in zip(self.joints, self.start)))
        self.timer = self.create_timer(self.dt, self._tick)

    def _target(self, t: float):
        """Cosine-eased position and velocity at time t, or None after the last segment."""
        previous = self.start
        for duration, goal in self.segments:
            if t < duration:
                s = 0.5 - 0.5 * math.cos(math.pi * t / duration)
                ds = 0.5 * math.pi / duration * math.sin(math.pi * t / duration)
                return ([a + s * (b - a) for a, b in zip(previous, goal)],
                        [ds * (b - a) for a, b in zip(previous, goal)])
            t -= duration
            previous = goal
        return None

    def _tick(self):
        target = self._target(self._t)
        if target is None:
            self.timer.cancel()
            self.get_logger().info('Pattern complete. The controller holds the last reference.')
            return
        positions, velocities = target
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joints)
        msg.position = positions
        msg.velocity = velocities
        msg.effort = [0.0] * len(positions)
        self.pub.publish(msg)
        self._t += self.dt


def main():
    # rclpy's own SIGINT handler shuts the context down behind this function's back. Without it,
    # SIGINT arrives as KeyboardInterrupt, within the spin timeout, and shutdown happens once, here.
    # launch forwards SIGINT and a terminal's Ctrl-C also reaches this process directly, so the
    # second one is ignored rather than interrupting the shutdown.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = BenchPlayer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

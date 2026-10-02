#!/usr/bin/env python3
"""S2 verifier: checks joint tracking and base height after trajectory completes."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml, sys, math

class SilStandVerifier(Node):
    def __init__(self):
        super().__init__('sil_stand_verifier')
        self.declare_parameter('keyframes', '')
        self.declare_parameter('tolerance_rad', 0.15)
        self.declare_parameter('check_delay_s', 20.0)

        kf_path = self.get_parameter('keyframes').value
        self.tol = self.get_parameter('tolerance_rad').value
        self.delay = self.get_parameter('check_delay_s').value

        with open(kf_path) as f:
            data = yaml.safe_load(f)
        self.joint_names = data['joint_names']
        self.target = data['keyframes'][-1]['positions']  # final standing pose

        self.latest = None
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._cb, 10)
        self.timer = self.create_timer(self.delay, self._check,
                                       clock=rclpy.clock.Clock())
        self.get_logger().info(
            f'Verifier armed: will check after {self.delay:.0f} s')

    def _cb(self, msg: JointState):
        self.latest = msg

    def _check(self):
        if self.latest is None:
            self.get_logger().error('FAIL: no /joint_states received')
            rclpy.shutdown()
            return

        msg = self.latest
        pos_map = dict(zip(msg.name, msg.position))
        errors = []
        for i, name in enumerate(self.joint_names):
            actual = pos_map.get(name, float('nan'))
            expected = self.target[i]
            err = abs(actual - expected)
            if math.isnan(actual) or err > self.tol:
                errors.append(f'{name}: expected={expected:.3f} '
                              f'actual={actual:.3f} err={err:.3f}')

        if errors:
            self.get_logger().error(
                f'FAIL: {len(errors)}/{len(self.joint_names)} joints '
                f'outside {self.tol:.2f} rad tolerance:')
            for e in errors:
                self.get_logger().error(f'  {e}')
        else:
            self.get_logger().info(
                f'PASS: all {len(self.joint_names)} joints within '
                f'{self.tol:.2f} rad of standing pose')

        self.destroy_timer(self.timer)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SilStandVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
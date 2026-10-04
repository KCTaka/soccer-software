#!/usr/bin/env python3
"""Runs the Unitree G1 walking policy and publishes joint references for the MIT controller.

Non-real-time. Reads the IMU (/imu_sensor_broadcaster/imu) and /joint_states, runs the ONNX policy
at `rate_hz`, and publishes to /humanoid_mit_controller/joint_references for all controlled joints.

Sequence: wait for fresh sensors, move every joint to the default stance over `ramp_s`, then run
the policy. The waist and arms are not policy outputs; they hold where they were at the start.
"""
import math
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

import walk_policy_core as core

_BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class WalkPolicy(Node):
    def __init__(self):
        super().__init__('walk_policy')
        declare = self.declare_parameter
        self.policy_path = declare('policy_path', '').value
        self.rate_hz = declare('rate_hz', 50.0).value
        self.policy_joints = list(declare('policy_joints', Parameter.Type.STRING_ARRAY).value)
        self.controlled = list(declare('controlled_joints', Parameter.Type.STRING_ARRAY).value)
        self.cfg = {
            'default_angles': np.array(declare('default_angles', Parameter.Type.DOUBLE_ARRAY).value),
            'ang_vel_scale': declare('ang_vel_scale', 1.0).value,
            'dof_pos_scale': declare('dof_pos_scale', 1.0).value,
            'dof_vel_scale': declare('dof_vel_scale', 1.0).value,
            'command_scale': np.array(declare('command_scale', Parameter.Type.DOUBLE_ARRAY).value),
        }
        self.action_scale = declare('action_scale', 1.0).value
        self.command_limit = np.array(declare('command_limit', Parameter.Type.DOUBLE_ARRAY).value)
        self.gait_period_s = declare('gait_period_s', 0.8).value
        self.ramp_s = declare('ramp_s', 3.0).value
        self.max_sensor_age_s = declare('max_sensor_age_s', 0.1).value
        self.cmd_vel_timeout_s = declare('cmd_vel_timeout_s', 0.5).value
        self.fallback_command = np.array(declare('fallback_command', Parameter.Type.DOUBLE_ARRAY).value)
        self.joint_limits_file = declare('joint_limits_file', '').value
        self.limit_margin_rad = declare('limit_margin_rad', 0.005).value

        if not self.policy_path:
            raise SystemExit('walk_policy: parameter policy_path is not set')
        n = len(self.policy_joints)
        if n == 0 or len(self.cfg['default_angles']) != n:
            raise SystemExit('walk_policy: default_angles must have one value per policy joint')
        missing = [j for j in self.policy_joints if j not in self.controlled]
        if missing:
            raise SystemExit(f'walk_policy: policy joints not in controlled_joints: {missing}')
        self.policy_index = [self.controlled.index(j) for j in self.policy_joints]
        self.limit_lower, self.limit_upper = self._load_limits()
        self.policy = core.Policy(self.policy_path)

        self.imu = None            # (receipt time s, quaternion xyzw, gyro)
        self.joints = None         # (receipt time s, {name: (position, velocity)})
        self.cmd = None            # (receipt time s, command)
        self.start = None          # joint positions when the ramp began, controlled order
        self.start_time = None
        self.step = 0
        self.last_action = np.zeros(n)
        self.running = False
        self.warned_stale = False

        self.pub = self.create_publisher(
            JointState, '/humanoid_mit_controller/joint_references', _BEST_EFFORT)
        self.create_subscription(Imu, '/imu_sensor_broadcaster/imu', self._on_imu, _BEST_EFFORT)
        self.create_subscription(JointState, '/joint_states', self._on_joints, _BEST_EFFORT)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 1)
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f'walk_policy: {self.policy_path}, {self.rate_hz:g} Hz, ramp {self.ramp_s:g} s')

    def _load_limits(self):
        """Position limits of the controlled joints from the generated safety manifest."""
        if not self.joint_limits_file:
            raise SystemExit('walk_policy: parameter joint_limits_file is not set')
        with open(self.joint_limits_file) as f:
            envelopes = yaml.safe_load(f)['envelopes']
        missing = [j for j in self.controlled if j not in envelopes]
        if missing:
            raise SystemExit(f'walk_policy: no position limits for {missing}')
        lower = np.array([envelopes[j]['position_min_rad'] for j in self.controlled])
        upper = np.array([envelopes[j]['position_max_rad'] for j in self.controlled])
        return lower + self.limit_margin_rad, upper - self.limit_margin_rad

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg):
        q = msg.orientation
        w = msg.angular_velocity
        values = [q.x, q.y, q.z, q.w, w.x, w.y, w.z]
        if all(math.isfinite(v) for v in values):
            self.imu = (self._now(), np.array(values[:4]), np.array(values[4:]))

    def _on_joints(self, msg):
        self.joints = (self._now(), {
            name: (msg.position[i], msg.velocity[i] if i < len(msg.velocity) else 0.0)
            for i, name in enumerate(msg.name)})

    def _on_cmd(self, msg):
        self.cmd = (self._now(), np.array([msg.linear.x, msg.linear.y, msg.angular.z]))

    def _fresh(self, sample):
        return sample is not None and self._now() - sample[0] <= self.max_sensor_age_s

    def _command(self):
        if self.cmd is not None and self._now() - self.cmd[0] <= self.cmd_vel_timeout_s:
            return core.clip_command(self.cmd[1], self.command_limit)
        return core.clip_command(self.fallback_command, self.command_limit)

    def _publish(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.controlled
        msg.position = [float(p) for p in positions]
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)
        self.pub.publish(msg)

    def _tick(self):
        if not (self._fresh(self.imu) and self._fresh(self.joints)):
            if self.running and not self.warned_stale:
                self.get_logger().error('sensor data is stale: holding the last reference')
                self.warned_stale = True
            return
        self.warned_stale = False
        names = self.joints[1]
        absent = [j for j in self.controlled if j not in names]
        if absent:
            self.get_logger().error(f'/joint_states lacks {absent}', throttle_duration_sec=5.0)
            return

        if self.start is None:
            self.start = np.array([names[j][0] for j in self.controlled])
            self.start_time = self._now()
            self.get_logger().info('sensors are live: moving to the default stance')

        goal = self.start.copy()
        goal[self.policy_index] = self.cfg['default_angles']
        elapsed = self._now() - self.start_time
        if elapsed < self.ramp_s:
            self._publish(core.ramp_targets(self.start, goal, elapsed, self.ramp_s))
            return

        if not self.running:
            self.running = True
            self.policy.reset()
            self.step = 0
            self.last_action = np.zeros(len(self.policy_joints))
            self.get_logger().info('policy running')

        _, quat, gyro = self.imu
        obs = core.build_observation(
            gyro, core.projected_gravity(quat), self._command(),
            [names[j][0] for j in self.policy_joints], [names[j][1] for j in self.policy_joints],
            self.last_action, core.gait_phase(self.step, self.rate_hz, self.gait_period_s),
            self.cfg)
        action = self.policy.step(obs)
        if not np.all(np.isfinite(action)):
            self.get_logger().error('policy produced a non-finite action: holding the stance',
                                    throttle_duration_sec=1.0)
            self._publish(goal)
            return
        self.last_action = action
        self.step += 1
        targets = goal.copy()
        targets[self.policy_index] = core.action_to_targets(
            action, self.cfg['default_angles'], self.action_scale)
        self._publish(core.clip_to_limits(targets, self.limit_lower, self.limit_upper))


def main():
    rclpy.init(args=sys.argv)
    node = WalkPolicy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

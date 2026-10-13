#!/usr/bin/env python3
"""Publish reference trajectories to MitImpedanceController for bench testing.

Runs in its own terminal against an already-running real bench (bench.launch.py sim:=false). It
captures the measured pose q0 from /joint_states, then streams sensor_msgs/JointState references
(position + velocity feed-forward) on /humanoid_mit_controller/joint_references, centring the active
joints' trajectory on their own q0 and holding every other joint at its q0. The trajectory math is
in bench_trajectories_core (pure, unit-tested); this node is the ROS wrapper.

  ros2 run humanoid_bringup bench_trajectories --ros-args -p mode:=sine -p joints:=[bench_j1] \
      -p amplitude_rad:=0.2 -p frequency_hz:=0.25
  ros2 run humanoid_bringup bench_trajectories --ros-args -p mode:=trap -p joints:=[bench_j1] \
      -p step_rad:=0.1

Only one reference source at a time: do not run this and the slider GUI together.
"""
import csv
import math
import os
import signal
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_trajectories_core as core  # noqa: E402

BENCH_JOINTS = ["bench_j1", "bench_j2", "bench_j3", "bench_j4", "bench_j5"]


class BenchTrajectories(Node):
    def __init__(self):
        super().__init__("bench_trajectories")
        p = self.declare_parameter
        self.topic = p("topic", "/humanoid_mit_controller/joint_references").value
        self.rate_hz = float(p("rate_hz", 200.0).value)
        self.mode = p("mode", "sine").value
        self.joints = list(p("joints", ["bench_j1"]).value)
        self.return_s = float(p("return_s", 2.0).value)
        self.record_csv = p("record_csv", "").value
        self.params = {
            "amplitude_rad": float(p("amplitude_rad", 0.2).value),
            "frequency_hz": float(p("frequency_hz", 0.25).value),
            "duration_s": float(p("duration_s", 20.0).value),
            "fade_s": float(p("fade_s", 2.0).value),
            "step_rad": float(p("step_rad", 0.1).value),
            "ramp_s": float(p("ramp_s", 2.0).value),
            "hold_s": float(p("hold_s", 2.0).value),
            "cycles": int(p("cycles", 3).value),
        }

        self._pub = self.create_publisher(JointState, self.topic, 1)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(
            String, "/robot_description", self._on_urdf,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self._measured = {}   # joint -> (pos, effort)
        self._limits = None   # joint -> (lower, upper), set when URDF arrives
        self.q0 = {}
        self._records = []    # (t, {joint: (cmd, meas, effort)})
        self._drift = {j: 0.0 for j in BENCH_JOINTS if j not in self.joints}  # non-active max drift

    # --- callbacks ---
    def _on_js(self, msg):
        eff = msg.effort if len(msg.effort) == len(msg.name) else [float("nan")] * len(msg.name)
        for name, pos, e in zip(msg.name, msg.position, eff):
            self._measured[name] = (pos, e)

    def _on_urdf(self, msg):
        limits = {}
        for joint in ET.fromstring(msg.data).findall("joint"):
            lim = joint.find("limit")
            if lim is not None and lim.get("lower") is not None and lim.get("upper") is not None:
                limits[joint.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
        self._limits = limits

    # --- startup ---
    def _wait(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def prepare(self):
        """Capture q0 and limits; validate joints and limits. Returns an error string, or None."""
        if not self._wait(lambda: all(j in self._measured for j in BENCH_JOINTS), 3.0):
            missing = [j for j in BENCH_JOINTS if j not in self._measured]
            return f"no /joint_states with all bench joints within 3 s (missing {missing}); " \
                   "is the bench running (bench.launch.py sim:=false)?"
        self.q0 = {j: self._measured[j][0] for j in BENCH_JOINTS}

        if not self._wait(lambda: self._limits is not None, 3.0):
            return "no /robot_description within 3 s; is robot_state_publisher running?"

        for j in self.joints:
            if j not in BENCH_JOINTS:
                return f"requested joint {j!r} is not a bench joint {BENCH_JOINTS}"
            if j not in self._limits:
                return f"no URDF <limit> for joint {j!r}; cannot bound the trajectory"

        traj = core.make_trajectory(self.mode, **self.params)
        for j in self.joints:
            lower, upper = self._limits[j]
            over = core.limit_violation(self.q0[j], lower, upper, traj.extent())
            if over > 0.0:
                return (f"joint {j!r}: trajectory centred on q0={self.q0[j]:.3f} would exceed its "
                        f"software limit [{lower:.3f}, {upper:.3f}] by {over:.3f} rad; "
                        "reduce amplitude_rad/step_rad or re-zero")
        self._traj = traj
        return None

    # --- run ---
    def _publish(self, deltas):
        """deltas: joint -> (pos_delta, vel). Non-active joints are held at q0 (delta 0, vel 0)."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(BENCH_JOINTS)
        for j in BENCH_JOINTS:
            d, v = deltas.get(j, (0.0, 0.0))
            pos = self.q0[j] + d
            if self._limits and j in self._limits:
                lo, hi = self._limits[j]
                pos = min(max(pos, lo), hi)
            msg.position.append(pos)
            msg.velocity.append(v)
        self._pub.publish(msg)
        return msg

    def run(self):
        dt = 1.0 / self.rate_hz
        duration = self._traj.duration()
        self._interrupted = False
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "_interrupted", True))

        last = {j: 0.0 for j in self.joints}
        t = 0.0
        next_tick = time.monotonic()
        while t <= duration and not self._interrupted and rclpy.ok():
            deltas = {}
            for j in self.joints:
                d, v = self._traj.point(t)
                deltas[j] = (d, v)
                last[j] = d
            msg = self._publish(deltas)
            self._record(t, deltas, msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.monotonic()))
            t += dt

        if self._interrupted:
            self.get_logger().info(f"interrupted: ramping back to q0 over {self.return_s:.1f} s")
            self._return_to_q0(last)
        else:
            self._publish({j: (0.0, 0.0) for j in self.joints})  # settle exactly at q0

    def _return_to_q0(self, last):
        """Ramp each active joint's reference delta from its current value to 0 over return_s."""
        dt = 1.0 / self.rate_hz
        steps = max(1, int(self.return_s / dt))
        next_tick = time.monotonic()
        for i in range(steps + 1):
            tau = i * dt
            deltas = {}
            for j in self.joints:
                pos, vel = core.cosine_ramp(tau, last[j], 0.0, self.return_s)
                deltas[j] = (pos, vel)
            self._publish(deltas)
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.monotonic()))

    # --- records + summary ---
    def _record(self, t, deltas, msg):
        row = {}
        pos_by_joint = dict(zip(msg.name, msg.position))
        for j in self.joints:
            meas, eff = self._measured.get(j, (float("nan"), float("nan")))
            row[j] = (pos_by_joint[j], meas, eff)
        self._records.append((t, row))
        for j in self._drift:  # non-active joints: accumulate the worst drift from q0
            if j in self._measured:
                self._drift[j] = max(self._drift[j], abs(self._measured[j][0] - self.q0[j]))

    def summary(self):
        if not self._records:
            self.get_logger().warning("no samples recorded")
            return
        self.get_logger().info("=== bench_trajectories summary ===")
        for j in self.joints:
            cmd = [r[j][0] for _, r in self._records]
            meas = [r[j][1] for _, r in self._records]
            eff = [r[j][2] for _, r in self._records]
            err = [m - c for c, m in zip(cmd, meas) if not math.isnan(m)]
            rms = math.sqrt(sum(e * e for e in err) / len(err)) if err else float("nan")
            mx = max((abs(e) for e in err), default=float("nan"))
            peak_eff = max((abs(e) for e in eff if not math.isnan(e)), default=float("nan"))
            # direction: do measured and commanded deltas (vs q0) share a sign on average?
            corr = sum((c - self.q0[j]) * (m - self.q0[j])
                       for c, m in zip(cmd, meas) if not math.isnan(m))
            self.get_logger().info(
                f"  {j}: rms_err={rms:.4f} max_err={mx:.4f} rad  peak|effort|={peak_eff:.3f}  "
                f"direction={'match' if corr > 0 else 'MISMATCH'}")
        worst = max(self._drift, key=self._drift.get, default=None)
        drift = self._drift.get(worst, 0.0) if worst else 0.0
        flag = "  <-- exceeds 0.01 rad" if drift > 0.01 else ""
        self.get_logger().info(f"  non-active max drift: {drift:.4f} rad ({worst}){flag}")

    def write_csv(self):
        if not self.record_csv:
            return
        with open(self.record_csv, "w", newline="") as f:
            w = csv.writer(f)
            cols = ["t"]
            for j in self.joints:
                cols += [f"{j}_cmd", f"{j}_meas", f"{j}_effort"]
            w.writerow(cols)
            for t, r in self._records:
                row = [f"{t:.4f}"]
                for j in self.joints:
                    row += [f"{v:.6f}" for v in r[j]]
                w.writerow(row)
        self.get_logger().info(f"wrote {self.record_csv} ({len(self._records)} samples)")


def main(args=None):
    rclpy.init(args=args)
    node = BenchTrajectories()
    try:
        error = node.prepare()
        if error:
            node.get_logger().error(error)
            node.destroy_node()
            rclpy.try_shutdown()
            sys.exit(1)
        node.run()
        node.summary()
        node.write_csv()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Planar four-bar linkages: the loops a robot model's `linkages` close.

A planar four-bar has a ground link carrying two revolute joints, the input (the crank a motor
turns) and the output (the joint controllers command); a coupler joint on the output's child link;
and a closure pin, which the kinematic tree cannot hold, joining the coupler's child link to the
input's child link. All four pin axes are parallel.

`FourBar` is the exact closed-form kinematics: given either crank's angle, the loop closes at an
intersection of two circles. HumanoidActuatorSystem's FourBarLinkage (C++) implements the same
equations from the parameters `planar_parameters` derives here and the URDF carries; the generator
uses this implementation to map the output joint's limits into the input joint's.

The planar frame has its origin at the output pivot O, x toward the input pivot P, and normal n
along the output joint's axis, so y = n x x. Angles are measured in this frame. A joint whose axis
is -n turns its link clockwise in it for positive motion, so its sign is -1.
"""
import math

import numpy as np

from .common import rpy_to_matrix

# Closure and alignment tolerances for a linkage at its zero pose. The model's numbers are CAD
# exports with ~1e-16 relative noise; anything above these is a modelling error, not rounding.
CLOSURE_TOL_M = 1e-6
PARALLEL_TOL = 1e-9

def zero_pose_frames(model):
    """link -> (R, p): every link frame in the root link's frame, every joint at 0."""
    joint_by_child = {j["child_link"]: j for j in model["joints"]}
    frames = {model["robot"]["root_link"]: (np.eye(3), np.zeros(3))}
    def frame(link):
        if link not in frames:
            j = joint_by_child[link]
            r_parent, p_parent = frame(j["parent_link"])
            frames[link] = (r_parent @ rpy_to_matrix(j["origin"]["rpy_rad"]),
                            p_parent + r_parent @ np.array(j["origin"]["xyz_m"], dtype=float))
        return frames[link]
    for l in model["links"]:
        frame(l["name"])
    return frames

def planar_parameters(model, linkage):
    """The linkage's planar four-bar parameters at the zero pose, and a list of what is wrong with
    the linkage (empty if it is a closed planar four-bar there). Lengths in m, angles in rad."""
    joints = {j["name"]: j for j in model["joints"]}
    j_in, j_out, j_cpl = (joints.get(linkage[k]) for k in
                          ("input_joint", "output_joint", "coupler_joint"))
    if None in (j_in, j_out, j_cpl):
        return None, [f"L-02: {linkage['name']}: a member joint is not declared"]
    errs = []
    if j_in["parent_link"] != j_out["parent_link"]:
        errs.append(f"L-02: {linkage['name']}: input and output joints do not share a parent link")
    if j_cpl["parent_link"] != j_out["child_link"]:
        errs.append(f"L-02: {linkage['name']}: the coupler joint's parent is not the output "
                    "joint's child")
    if any(j["type"] != "revolute" for j in (j_in, j_out, j_cpl)):
        errs.append(f"L-02: {linkage['name']}: every member joint must be revolute")
    if errs:
        return None, errs

    frames = zero_pose_frames(model)
    r_g, p_g = frames[j_out["parent_link"]]
    def in_ground(link, point):
        r, p = frames[link]
        return r_g.T @ (p + r @ np.asarray(point, dtype=float) - p_g)
    def axis_in_ground(j):
        r, _ = frames[j["child_link"]]
        return r_g.T @ r @ np.asarray(j["axis"], dtype=float)

    n = axis_in_ground(j_out)
    signs = {}
    for key, j in (("input", j_in), ("output", j_out), ("coupler", j_cpl)):
        a = axis_in_ground(j)
        if np.linalg.norm(np.cross(a, n)) > PARALLEL_TOL:
            errs.append(f"L-02: {linkage['name']}: {j['name']} axis is not parallel to the "
                        "output axis")
        signs[key] = 1 if a @ n > 0 else -1

    o = in_ground(j_out["child_link"], [0, 0, 0])
    p = in_ground(j_in["child_link"], [0, 0, 0])
    b = in_ground(j_cpl["child_link"], [0, 0, 0])
    t_cpl = in_ground(j_cpl["child_link"], linkage["closure"]["coupler_point_m"])
    t_in = in_ground(j_in["child_link"], linkage["closure"]["input_point_m"])
    closure_error = float(np.linalg.norm(t_cpl - t_in))
    if closure_error > CLOSURE_TOL_M:
        errs.append(f"L-03: {linkage['name']}: the closure pin's two points are "
                    f"{closure_error * 1e3:.4f} mm apart at the zero pose, which must close the "
                    "loop")

    # Project onto the mechanism plane through O.
    x = p - o
    x -= (x @ n) * n
    if np.linalg.norm(x) < CLOSURE_TOL_M:
        return None, errs + [f"L-02: {linkage['name']}: input and output pivots coincide in the "
                             "plane"]
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    def planar(v):
        d = v - o
        return np.array([d @ x, d @ y])
    P, B, T = planar(p), planar(b), planar(t_cpl)
    params = {
        "ground_length_m": float(P[0]),
        "input_crank_length_m": float(np.linalg.norm(T - P)),
        "coupler_length_m": float(np.linalg.norm(T - B)),
        "output_crank_length_m": float(np.linalg.norm(B)),
        "input_zero_angle_rad": math.atan2(*(T - P)[::-1]),
        "output_zero_angle_rad": math.atan2(*B[::-1]),
        "coupler_zero_angle_rad": math.atan2(*(T - B)[::-1]) - math.atan2(*B[::-1]),
        "input_axis_sign": signs["input"],
        "output_axis_sign": signs["output"],
        "coupler_axis_sign": signs["coupler"],
    }
    return params, errs

def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def _circles(c0, r0, c1, r1, side):
    """The intersection of circle (c0, r0) and circle (c1, r1) on `side` (+1 left of c0->c1), or
    None if they do not meet (the linkage cannot close there)."""
    d = c1 - c0
    dist = math.hypot(*d)
    along = (dist * dist + r0 * r0 - r1 * r1) / (2 * dist)
    h2 = r0 * r0 - along * along
    if h2 < 0:
        return None
    u = d / dist
    return c0 + along * u + side * math.sqrt(h2) * np.array([-u[1], u[0]])

def _cross(u, v):
    return u[0] * v[1] - u[1] * v[0]

class FourBar:
    """Exact closed-form kinematics of a planar four-bar given by `planar_parameters`. The
    assembly mode (which circle intersection) is the one the zero pose closes in; it cannot change
    without passing a toggle, where the ratio is unbounded."""

    def __init__(self, params):
        self.p = params
        self.P = np.array([params["ground_length_m"], 0.0])
        self._side_from_output = self._side(lambda s: self._input_tip_from_output(0.0, s),
                                            self._input_tip(0.0))
        self._side_from_input = self._side(lambda s: self._output_tip_from_input(0.0, s),
                                           self._output_tip(0.0))

    @staticmethod
    def _side(solve, expected):
        for side in (1, -1):
            got = solve(side)
            if got is not None and np.linalg.norm(got - expected) < CLOSURE_TOL_M:
                return side
        raise ValueError("the four-bar does not close at its zero pose")

    def _input_angle(self, q_in):
        return self.p["input_zero_angle_rad"] + self.p["input_axis_sign"] * q_in

    def _output_angle(self, q_out):
        return self.p["output_zero_angle_rad"] + self.p["output_axis_sign"] * q_out

    def _input_tip(self, q_in):
        a = self._input_angle(q_in)
        return self.P + self.p["input_crank_length_m"] * np.array([math.cos(a), math.sin(a)])

    def _output_tip(self, q_out):
        a = self._output_angle(q_out)
        return self.p["output_crank_length_m"] * np.array([math.cos(a), math.sin(a)])

    def _input_tip_from_output(self, q_out, side):
        return _circles(self.P, self.p["input_crank_length_m"], self._output_tip(q_out),
                        self.p["coupler_length_m"], side)

    def _output_tip_from_input(self, q_in, side):
        return _circles(np.zeros(2), self.p["output_crank_length_m"], self._input_tip(q_in),
                        self.p["coupler_length_m"], side)

    def _configuration(self, t, b):
        """(q_in, q_out, q_cpl, ratio dq_out/dq_in, ratio dq_cpl/dq_out) at input tip t and output
        tip b."""
        e_in, e_out, r = t - self.P, b, t - b
        q_in = self.p["input_axis_sign"] * _wrap(math.atan2(e_in[1], e_in[0]) -
                                                 self.p["input_zero_angle_rad"])
        th_out = math.atan2(e_out[1], e_out[0])
        q_out = self.p["output_axis_sign"] * _wrap(th_out - self.p["output_zero_angle_rad"])
        q_cpl = self.p["coupler_axis_sign"] * _wrap(
            math.atan2(r[1], r[0]) - th_out - self.p["coupler_zero_angle_rad"])
        # Velocity closure (t - b) . (v_t - v_b) = 0 with v = omega x crank gives the planar rates.
        dth_out_dth_in = _cross(e_in, r) / _cross(e_out, r)
        ratio = self.p["output_axis_sign"] * self.p["input_axis_sign"] * dth_out_dth_in
        # Coupler absolute rate per unit output rate, then relative to the output link.
        v_t = np.array([-e_in[1], e_in[0]]) / dth_out_dth_in
        v_b = np.array([-e_out[1], e_out[0]])
        dphi = _cross(r, v_t - v_b) / (r @ r)
        cpl_per_out = self.p["coupler_axis_sign"] * self.p["output_axis_sign"] * (dphi - 1.0)
        return q_in, q_out, q_cpl, ratio, cpl_per_out

    def from_output(self, q_out):
        """(q_in, q_cpl, dq_out/dq_in, dq_cpl/dq_out) with the output at q_out, or None where the
        linkage cannot close."""
        b = self._output_tip(q_out)
        t = self._input_tip_from_output(q_out, self._side_from_output)
        if t is None:
            return None
        q_in, _, q_cpl, ratio, cpl = self._configuration(t, b)
        return q_in, q_cpl, ratio, cpl

    def from_input(self, q_in):
        """(q_out, q_cpl, dq_out/dq_in, dq_cpl/dq_out) with the input at q_in, or None."""
        t = self._input_tip(q_in)
        b = self._output_tip_from_input(q_in, self._side_from_input)
        if b is None:
            return None
        _, q_out, q_cpl, ratio, cpl = self._configuration(t, b)
        return q_out, q_cpl, ratio, cpl

    def sweep(self, lower, upper, samples=2001):
        """from_output at `samples` points spanning [lower, upper], or None if the linkage cannot
        close somewhere in it or the ratio changes sign (a toggle lies inside)."""
        out = []
        for q in np.linspace(lower, upper, samples):
            c = self.from_output(float(q))
            if c is None:
                return None
            out.append((float(q),) + c)
        ratios = np.array([c[3] for c in out])
        if not (np.all(ratios > 0) or np.all(ratios < 0)):
            return None
        return out

# Linkage-derived limits are rounded so that a last-bit difference between two machines' libm
# cannot change a generated file.
DERIVED_DECIMALS = 10

def members(model):
    """Names of every joint a linkage drives rather than a controller: its inputs and couplers."""
    return {lk[k] for lk in model.get("linkages", []) for k in ("input_joint", "coupler_joint")}

def commanded_joints(model):
    """The joints controllers command, in model order: every non-fixed joint except a linkage's
    input and coupler."""
    passive = members(model)
    return [j for j in model["joints"] if j["type"] != "fixed" and j["name"] not in passive]

def drives(model):
    """For each commanded joint, in model order, how its command reaches a motor: a dict with
      joint           the commanded joint
      actuated_joint  the joint the actuator turns; the transport and the safety manifest name the
                      motor by it. The joint itself unless a linkage drives it.
      actuator        the actuator's name
      linkage, four_bar   the linkage and its kinematics, or None
      limits          the actuated joint's position range, velocity and torque bounds:
                      position_min/max_rad, velocity_rad_s, effort_continuous_nm, effort_peak_nm,
                      effort_peak_duration_s. Through a linkage they are derived over the output's
                      software range: the input range is the image of it (the map is monotone),
                      and the input's speed and torque bounds are what the output's imply at the
                      worst ratio in that range.
      coupler         through a linkage, the coupler joint's range and speed bound over it
    Raises ValueError for a linkage that does not close over the output's software range."""
    joints = {j["name"]: j for j in model["joints"]}
    by_output = {lk["output_joint"]: lk for lk in model.get("linkages", [])}
    out = []
    for j in commanded_joints(model):
        lim = j["limits"]
        lk = by_output.get(j["name"])
        if lk is None:
            out.append({"joint": j["name"], "actuated_joint": j["name"], "actuator": j["actuator"],
                        "linkage": None, "four_bar": None, "coupler": None, "limits": {
                            "position_min_rad": lim["software_lower_rad"],
                            "position_max_rad": lim["software_upper_rad"],
                            "velocity_rad_s": lim["velocity_rad_s"],
                            "effort_continuous_nm": lim["effort_continuous_nm"],
                            "effort_peak_nm": lim["effort_peak_nm"],
                            "effort_peak_duration_s": lim["effort_peak_duration_s"]}})
            continue
        params, errs = planar_parameters(model, lk)
        if errs:
            raise ValueError("; ".join(errs))
        four_bar = FourBar(params)
        sweep = four_bar.sweep(lim["software_lower_rad"], lim["software_upper_rad"])
        if sweep is None:
            raise ValueError(f"{lk['name']}: the linkage does not close, or passes a toggle, "
                             f"within {j['name']}'s software range")
        q_in = [s[1] for s in sweep]
        q_cpl = [s[2] for s in sweep]
        ratio = [abs(s[3]) for s in sweep]
        cpl_rate = [abs(s[4]) for s in sweep]
        r = lambda x: round(float(x), DERIVED_DECIMALS)
        out.append({
            "joint": j["name"], "actuated_joint": lk["input_joint"],
            "actuator": joints[lk["input_joint"]]["actuator"], "linkage": lk,
            "four_bar": four_bar,
            "limits": {
                "position_min_rad": r(min(q_in)), "position_max_rad": r(max(q_in)),
                "velocity_rad_s": r(lim["velocity_rad_s"] / min(ratio)),
                "effort_continuous_nm": r(lim["effort_continuous_nm"] * max(ratio)),
                "effort_peak_nm": r(lim["effort_peak_nm"] * max(ratio)),
                "effort_peak_duration_s": lim["effort_peak_duration_s"]},
            "coupler": {"joint": lk["coupler_joint"], "position_min_rad": r(min(q_cpl)),
                        "position_max_rad": r(max(q_cpl)),
                        "velocity_rad_s": r(lim["velocity_rad_s"] * max(cpl_rate))},
        })
    return out

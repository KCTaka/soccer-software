"""Usage:
  python3 -m model.generators.cli generate [--robot NAME]
  python3 -m model.generators.cli check    [--robot NAME] [--mj-smoke]

Robots are declared in model/robots.yaml. With no --robot, `check` runs every robot and
`generate` uses the registry's default_robot. Each robot's robot_model.yaml is the hand-edited
source of truth; `check` validates it and diffs the generated files against a regeneration.

Each robot's description goes to src/humanoid_description/robots/<robot>/, the files other
repositories import to model/robots/<robot>/generated/. Both are wholly generated: `generate`
replaces them, and `check` fails on a file that differs, is missing or is no longer generated.
"""
import argparse, filecmp, os, shutil, sys, tempfile
import yaml

from . import emit, validate

REGISTRY = "model/robots.yaml"
SCHEMA = "model/schema/robot_model.schema.json"
MOTORS = "model/motors.yaml"
MOTORS_SCHEMA = "model/schema/motors.schema.json"
DESCRIPTIONS = "src/humanoid_description/robots"

def _registry():
    with open(REGISTRY) as f:
        return yaml.safe_load(f)

def _robot(name):
    reg = _registry()
    robots = reg["robots"]
    if name is None:
        name = reg["default_robot"]
    if name not in robots:
        sys.exit(f"unknown robot {name!r}; known: {', '.join(sorted(robots))}")
    r = dict(robots[name])
    r["name"] = name
    return r

def _motor_limits():
    with open(MOTORS) as f:
        return yaml.safe_load(f)

def _all_robots():
    reg = _registry()
    return [dict(reg["robots"][n], name=n) for n in reg["robots"]]

def _outputs(r):
    """The robot's two generated directories in the repository: its description, and its exports."""
    return (os.path.join(DESCRIPTIONS, r["name"]),
            os.path.join("model", "robots", r["name"], "generated"))

def _generate(r, motor_limits, root):
    """Generate the robot into `root`, laid out as the repository holds it: the outputs' paths
    under it are _outputs(r)."""
    description_dir, export_dir = (os.path.join(root, d) for d in _outputs(r))
    emit.generate(r["name"], r["source"], r["overlay"], motor_limits, description_dir, export_dir,
                  r["assets_dir"], urdf=r["urdf"], mjcf=r["mjcf"], license_path=r.get("license"),
                  firmware_config=r.get("firmware_config", False),
                  firmware_setup=r.get("firmware_setup"),
                  stm32_wiring=r.get("stm32_wiring", False))

def cmd_generate(args):
    # Generated aside, then swapped in, so a failed run leaves the repository as it was and no file
    # a previous run wrote outlives the run that stopped writing it.
    r = _robot(args.robot)
    with tempfile.TemporaryDirectory() as td:
        _generate(r, _motor_limits(), td)
        for d in _outputs(r):
            shutil.rmtree(d, ignore_errors=True)
            if os.path.isdir(os.path.join(td, d)):
                shutil.copytree(os.path.join(td, d), d)

def _files(d):
    return {os.path.relpath(os.path.join(p, f), d)
            for p, _, fs in os.walk(d) for f in fs} if os.path.isdir(d) else set()

def _tree_drift(committed, regenerated):
    """How the committed tree differs from a regeneration, one message per file."""
    have, want = _files(committed), _files(regenerated)
    errs = [f"{committed}/{f} is not generated any more" for f in sorted(have - want)]
    errs += [f"{committed}/{f} is missing" for f in sorted(want - have)]
    errs += [f"{committed}/{f} differs from regeneration" for f in sorted(have & want)
             if not filecmp.cmp(os.path.join(committed, f), os.path.join(regenerated, f),
                                shallow=False)]
    return errs

def _mj_smoke(scene):
    """The scene loads, the robot starts on the floor rather than in it at qpos0 and at every
    keyframe, and 100 steps from qpos0 stay finite."""
    import mujoco
    import numpy as np
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    for k in range(m.nkey):
        mujoco.mj_resetDataKeyframe(m, d, k)
        mujoco.mj_forward(m, d)
        depth = max((-c.dist for c in d.contact[:d.ncon]), default=0.0)
        if depth > 1e-6:
            return [f"the robot starts {depth * 1000:.1f} mm inside the floor at keyframe "
                    f"{m.key(k).name}"]
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)
    ncon0 = d.ncon
    depth = max((-c.dist for c in d.contact[:ncon0]), default=0.0)
    if depth > 1e-6:
        return [f"the robot starts {depth * 1000:.1f} mm inside the floor at qpos0"]
    for _ in range(100):
        mujoco.mj_step(m, d)
    if not np.isfinite(d.qpos).all():
        return ["non-finite qpos after 100 steps"]
    print(f"mj-smoke OK: nq={m.nq}, nv={m.nv}, nbody={m.nbody}, nkey={m.nkey}, "
          f"ncon at qpos0={ncon0}")
    return []

def _check_one(r, motor_limits, mj_smoke):
    print(f"--- {r['name']} ---")
    with open(r["source"]) as f:
        model = yaml.safe_load(f)
    errs = validate.schema_validate(model, SCHEMA)
    errs += validate.structural_checks(model, motor_limits)
    if errs:
        for e in errs:
            print(f"ERROR: {e}", file=sys.stderr)
        return False

    mjcf = os.path.join(_outputs(r)[0], "mjcf", "scene.xml")
    if mj_smoke and os.path.exists(mjcf):
        smoke_errs = _mj_smoke(mjcf)
        for e in smoke_errs:
            print(f"ERROR: mj-smoke: {e}", file=sys.stderr)
        if smoke_errs:
            return False

    # ADR-009-07: regenerate into a temp dir and diff.
    with tempfile.TemporaryDirectory() as td:
        _generate(r, motor_limits, td)
        drift = [e for d in _outputs(r) for e in _tree_drift(d, os.path.join(td, d))]
    for e in drift:
        print(f"ERROR: drift: {e} (run `generate --robot {r['name']}`)", file=sys.stderr)
    if drift:
        return False
    print(f"check OK: {r['name']}")
    return True

def cmd_check(args):
    motor_limits = _motor_limits()
    errs = validate.schema_validate(motor_limits, MOTORS_SCHEMA)
    for e in errs:
        print(f"ERROR: {MOTORS}: {e}", file=sys.stderr)
    robots = [_robot(args.robot)] if args.robot else _all_robots()
    ok = not errs and all(_check_one(r, motor_limits, args.mj_smoke) for r in robots)
    if not ok:
        sys.exit(1)
    print("check OK: all robots pass schema, structural checks, and regenerate-and-diff")

def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("generate")
    g.add_argument("--robot", default=None)
    g.set_defaults(fn=cmd_generate)
    c = sub.add_parser("check")
    c.add_argument("--robot", default=None)
    c.add_argument("--mj-smoke", action="store_true")
    c.set_defaults(fn=cmd_check)
    args = p.parse_args()
    args.fn(args)

if __name__ == "__main__":
    main()

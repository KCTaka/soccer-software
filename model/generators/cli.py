"""Usage:
  python3 -m model.generators.cli generate [--robot NAME]
  python3 -m model.generators.cli check    [--robot NAME] [--mj-smoke]

Robots are declared in model/robots.yaml. With no --robot, `check` runs every robot and
`generate` uses the registry's default_robot. Each robot's robot_model.yaml is the hand-edited
source of truth; `check` validates it and diffs the generated files against a regeneration.
"""
import argparse, filecmp, os, sys, tempfile
import yaml

from . import emit, validate

REGISTRY = "model/robots.yaml"
SCHEMA = "model/schema/robot_model.schema.json"
MOTORS = "model/motors.yaml"
MOTORS_SCHEMA = "model/schema/motors.schema.json"

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

def _generate(r, motor_limits, out_dir):
    emit.generate(r["source"], r["overlay"], motor_limits, out_dir, r["generated"], r["assets_dir"],
                  urdf=r["urdf"], mjcf=r["mjcf"],
                  firmware_config=r.get("firmware_config", False),
                  firmware_setup=r.get("firmware_setup"),
                  stm32_wiring=r.get("stm32_wiring", False),
                  safety_manifest_path=r.get("safety_manifest_path"),
                  safety_manifest_package=r.get("safety_manifest_package"))

def cmd_generate(args):
    r = _robot(args.robot)
    _generate(r, _motor_limits(), r["generated"])

def _check_one(r, motor_limits, mj_smoke):
    print(f"--- {r['name']} ---")
    with open(r["source"]) as f:
        model = yaml.safe_load(f)
    errs = validate.schema_validate(model, SCHEMA)
    s_errs, warns = validate.structural_checks(model, motor_limits)
    errs += s_errs
    for w in warns:
        print(f"WARN: {w}")
    if errs:
        for e in errs:
            print(f"ERROR: {e}", file=sys.stderr)
        return False

    mjcf = os.path.join(r["generated"], "robot.mjcf")
    if mj_smoke and os.path.exists(mjcf):
        try:
            import mujoco, math
            m = mujoco.MjModel.from_xml_path(mjcf)
            d = mujoco.MjData(m)
            for _ in range(100):
                mujoco.mj_step(m, d)
            assert all(math.isfinite(x) for x in d.qpos), "NaN in qpos"
            print(f"mj-smoke OK: nq={m.nq}, nv={m.nv}, nbody={m.nbody}")
        except ImportError:
            print("WARN: mujoco not installed; skipping smoke test")

    # ADR-009-07: regenerate into a temp dir and diff.
    with tempfile.TemporaryDirectory() as td:
        t_out = os.path.join(td, "gen")
        _generate(r, motor_limits, t_out)
        for f in os.listdir(t_out):
            if not filecmp.cmp(os.path.join(r["generated"], f), os.path.join(t_out, f),
                               shallow=False):
                print(f"ERROR: drift: {r['generated']}/{f} differs from regeneration",
                      file=sys.stderr)
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

"""Usage:
  python3 -m model.generators.cli ingest
  python3 -m model.generators.cli generate
  python3 -m model.generators.cli check [--mj-smoke]
"""
import argparse, filecmp, os, shutil, subprocess, sys, tempfile
import yaml

from . import emit, ingest, validate

CONFIG = "model/source/ingest.yaml"
SRC = "model/source/robot_model.yaml"
OVERLAY = "model/source/menagerie_contact_overlay.yaml"
OUT = "model/generated"
SCHEMA = "model/schema/robot_model.schema.json"

def _cfg():
    with open(CONFIG) as f:
        return yaml.safe_load(f)

def cmd_ingest(_):
    c = _cfg()
    ingest.ingest(CONFIG, SRC, OVERLAY)

def cmd_generate(_):
    c = _cfg()
    emit.generate(SRC, OVERLAY, OUT, c["source"]["assets_dir"])

def cmd_check(args):
    with open(SRC) as f:
        model = yaml.safe_load(f)
    errs = validate.schema_validate(model, SCHEMA)
    s_errs, warns, total = validate.structural_checks(model)
    errs += s_errs
    for w in warns:
        print(f"WARN: {w}")
    if errs:
        for e in errs:
            print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    if args.mj_smoke:
        try:
            import mujoco
            m = mujoco.MjModel.from_xml_path(os.path.join(OUT, "robot.mjcf"))
            d = mujoco.MjData(m)
            for _ in range(100):
                mujoco.mj_step(m, d)
            import math
            assert all(math.isfinite(x) for x in d.qpos), "NaN in qpos"
            print(f"mj-smoke OK: nq={m.nq}, nv={m.nv}, nbody={m.nbody}")
        except ImportError:
            print("WARN: mujoco not installed; skipping smoke test")
    # ADR-009-07: regenerate into a temp dir and diff.
    with tempfile.TemporaryDirectory() as td:
        t_src, t_ovl = os.path.join(td, "rm.yaml"), os.path.join(td, "ov.yaml")
        ingest.ingest(CONFIG, t_src, t_ovl)
        if not filecmp.cmp(SRC, t_src, shallow=False):
            sys.exit("drift: robot_model.yaml differs from regeneration")
        t_out = os.path.join(td, "gen")
        c = _cfg()
        emit.generate(t_src, t_ovl, t_out, c["source"]["assets_dir"])
        for f in os.listdir(t_out):
            if not filecmp.cmp(os.path.join(OUT, f), os.path.join(t_out, f), shallow=False):
                sys.exit(f"drift: generated/{f} differs from regeneration")
    print("check OK: schema, structural checks, and regenerate-and-diff all pass")

def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ingest").set_defaults(fn=cmd_ingest)
    sub.add_parser("generate").set_defaults(fn=cmd_generate)
    c = sub.add_parser("check")
    c.add_argument("--mj-smoke", action="store_true")
    c.set_defaults(fn=cmd_check)
    args = p.parse_args()
    args.fn(args)

if __name__ == "__main__":
    main()
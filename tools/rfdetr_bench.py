#!/usr/bin/env python3
"""Measure RF-DETR on the actual Jetson (localization report §3.2).

Every published RF-DETR latency number is NVIDIA T4, FP16, batch 1 — a datacenter
card. The report explicitly defers the detector decision until the candidate is
benchmarked on the real Orin, because attention-heavy transformers historically
lose ground to well-optimized CNNs on Jetson-class hardware. This script produces
that number.

Per requested size it: exports the COCO-pretrained checkpoint to ONNX (CPU torch),
builds an FP16 TensorRT engine with trtexec against the SAME TensorRT the ZED image
ships, then benchmarks it and prints a table of median/p99 GPU latency, throughput
and engine size.

Run inside soccer-rfdetr-bench:jazzy (see deploy/docker/Dockerfile.rfdetr-bench).
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

VENV_PY = "/opt/rfdetr-venv/bin/python3"

# Input resolutions are fixed per size by the architecture search (README table).
SIZES: dict[str, dict] = {
    "nano": {"cls": "RFDETRNano", "res": 384},
    "small": {"cls": "RFDETRSmall", "res": 512},
    "medium": {"cls": "RFDETRMedium", "res": 576},
    "large": {"cls": "RFDETRLarge", "res": 704},
}

# Exporting imports torch, so it runs in the venv as a subprocess rather than here.
_EXPORT_SNIPPET = """
import sys, glob, os
from rfdetr import {cls}
out = "{out}"
os.makedirs(out, exist_ok=True)
model = {cls}()
model.export(output_dir=out)
found = sorted(glob.glob(os.path.join(out, "*.onnx")))
print("ONNX_FILES=" + ",".join(found))
"""


def _run(cmd: list[str], log: Path) -> tuple[int, str]:
    proc = subprocess.run(cmd, capture_output=True, text=True)
    output = proc.stdout + proc.stderr
    log.write_text(output)
    return proc.returncode, output


def export_onnx(size: str, workdir: Path) -> Path | None:
    cfg = SIZES[size]
    out_dir = workdir / f"rfdetr_{size}"
    existing = sorted(out_dir.glob("*.onnx"))
    if existing:
        print(f"  [export] reusing {existing[0].name}")
        return existing[0]

    print(f"  [export] {cfg['cls']} -> ONNX (CPU torch, downloads weights on first run)")
    started = time.time()
    code, output = _run(
        [VENV_PY, "-c", _EXPORT_SNIPPET.format(cls=cfg["cls"], out=out_dir)],
        workdir / f"export_{size}.log",
    )
    if code != 0:
        print(f"  [export] FAILED (rc={code}); tail:")
        print("    " + "\n    ".join(output.strip().splitlines()[-15:]))
        return None

    match = re.search(r"ONNX_FILES=(.*)", output)
    files = [Path(p) for p in match.group(1).split(",") if p] if match else []
    if not files:
        print("  [export] FAILED: export produced no .onnx")
        return None
    print(f"  [export] ok in {time.time() - started:.0f}s -> {files[0].name}"
          f" ({files[0].stat().st_size / 1e6:.0f} MB)")
    return files[0]


def build_and_bench(size: str, onnx: Path, workdir: Path) -> dict | None:
    """Build an FP16 engine and benchmark it. trtexec does both in one pass."""
    cfg = SIZES[size]
    engine = workdir / f"rfdetr_{size}_fp16.engine"
    res = cfg["res"]
    cmd = [
        "trtexec",
        f"--onnx={onnx}",
        f"--saveEngine={engine}",
        "--fp16",
        "--noDataTransfers",     # measure pure GPU compute, not PCIe/host copies
        "--useCudaGraph",        # how a real detector node would dispatch it
        "--warmUp=2000",
        "--duration=20",
        "--avgRuns=100",
        "--separateProfileRun",
        # Harmless when the ONNX has static shapes; required when it does not.
        f"--shapes=input:1x3x{res}x{res}",
    ]
    print(f"  [trtexec] building FP16 engine @ {res}x{res} and benchmarking (~1-3 min)")
    started = time.time()
    code, output = _run(cmd, workdir / f"trtexec_{size}.log")
    if code != 0:
        # Retry without --shapes: fails hard if the ONNX input is not named "input".
        print("  [trtexec] retrying without explicit --shapes")
        code, output = _run(
            [c for c in cmd if not c.startswith("--shapes")],
            workdir / f"trtexec_{size}.log",
        )
    if code != 0:
        print(f"  [trtexec] FAILED (rc={code}); tail:")
        print("    " + "\n    ".join(output.strip().splitlines()[-20:]))
        return None

    def grab(pattern: str) -> float | None:
        m = re.search(pattern, output)
        return float(m.group(1)) if m else None

    result = {
        "size": size,
        "resolution": f"{res}x{res}",
        "build_s": round(time.time() - started),
        "engine_mb": round(engine.stat().st_size / 1e6, 1) if engine.exists() else None,
        "throughput_qps": grab(r"Throughput:\s*([\d.]+)\s*qps"),
        "median_ms": grab(r"GPU Compute Time:.*?median = ([\d.]+) ms"),
        "mean_ms": grab(r"GPU Compute Time:.*?mean = ([\d.]+) ms"),
        "p99_ms": grab(r"GPU Compute Time:.*?percentile\(99(?:\.00)?%\) = ([\d.]+) ms"),
        "latency_ms": grab(r"Latency:.*?median = ([\d.]+) ms"),
    }
    print(f"  [trtexec] ok: {result['median_ms']} ms median, "
          f"{result['throughput_qps']} qps, engine {result['engine_mb']} MB")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sizes", nargs="+", default=["nano"], choices=list(SIZES))
    parser.add_argument("--workdir", type=Path, default=Path("/models"))
    args = parser.parse_args()

    if not shutil.which("trtexec"):
        print("trtexec not found — run this inside soccer-rfdetr-bench:jazzy.")
        return 2
    args.workdir.mkdir(parents=True, exist_ok=True)

    results = []
    for size in args.sizes:
        print(f"\n=== RF-DETR {size} ({SIZES[size]['res']}px) ===")
        onnx = export_onnx(size, args.workdir)
        if onnx is None:
            continue
        result = build_and_bench(size, onnx, args.workdir)
        if result:
            results.append(result)

    if not results:
        print("\nNo results.")
        return 1

    print("\n" + "=" * 78)
    print("RF-DETR on Jetson Orin Nano Super — TensorRT FP16, batch 1")
    print("=" * 78)
    header = f"{'size':<8}{'input':<10}{'median':>9}{'p99':>9}{'qps':>9}{'engine':>9}"
    print(header)
    print("-" * 78)
    for r in results:
        print(f"{r['size']:<8}{r['resolution']:<10}"
              f"{r['median_ms'] or 0:>8.2f}m{r['p99_ms'] or 0:>8.2f}m"
              f"{r['throughput_qps'] or 0:>9.1f}{r['engine_mb'] or 0:>8.0f}M")
    print("-" * 78)
    print("median/p99 = GPU compute time in ms; qps = end-to-end throughput.")

    out = args.workdir / "rfdetr_bench_results.json"
    out.write_text(json.dumps(results, indent=2))
    print(f"\nWrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Static invariant checks for the soccer-bot repository.

Each check corresponds to a bug that reached the robot and was expensive to
diagnose at runtime. They are cheap, need no ROS, no GPU and no Docker, so they
run in cloud CI and via ``make check``.

    python3 tools/check_repo_invariants.py

Exit code 0 = all invariants hold, 1 = at least one violation.
"""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

failures: list[str] = []
passes: list[str] = []


def ok(msg: str) -> None:
    passes.append(msg)


def bad(check: str, msg: str, fix: str) -> None:
    failures.append(f"{check}\n    {msg}\n    fix: {fix}")


# ---------------------------------------------------------------------------
# 1. Multi-stage runtime images must install their own runtime dependencies.
# ---------------------------------------------------------------------------
# soccer-app:jazzy shipped with no ROS at all: the final stage was FROM a base
# that only carried the ROS *apt repository*, and every dependency had been
# installed in the discarded build stage. The image copied its compiled
# install/ space but nothing it links against, so it could not run its own CMD.
#
# Invariant: a stage that copies a colcon install/ space out of another stage
# is a runtime image, and must apply the shared dependency manifest itself.

STAGE_RE = re.compile(r"^\s*FROM\s+(\S+)(?:\s+AS\s+(\S+))?", re.IGNORECASE)
COPY_INSTALL_RE = re.compile(r"^\s*COPY\s+--from=(\S+)\s+\S*install\b", re.IGNORECASE)
DEPS_FILE = "soccer-app-deps.apt"


def check_dockerfile_runtime_deps() -> None:
    for dockerfile in sorted((REPO / "deploy" / "docker").glob("Dockerfile*")):
        # Split into stages, keeping each stage's body.
        stages: list[tuple[str, list[str]]] = []
        for line in dockerfile.read_text().splitlines():
            m = STAGE_RE.match(line)
            if m:
                stages.append((m.group(2) or m.group(1), []))
            elif stages:
                stages[-1][1].append(line)

        for name, body in stages:
            copies_install = any(COPY_INSTALL_RE.match(ln) for ln in body)
            if not copies_install:
                continue
            applies_deps = any(DEPS_FILE in ln for ln in body)
            rel = dockerfile.relative_to(REPO)
            if applies_deps:
                ok(f"{rel}: stage '{name}' copies install/ and applies {DEPS_FILE}")
            else:
                bad(
                    "docker-runtime-deps",
                    f"{rel}: stage '{name}' copies a colcon install/ space but never "
                    f"installs {DEPS_FILE}, so the image may have no ROS runtime.",
                    f"add a COPY + apt install of deploy/docker/{DEPS_FILE} to that "
                    "stage, before the COPY --from=... install/ line.",
                )


# ---------------------------------------------------------------------------
# 2. camera.launch.py must start a robot_state_publisher.
# ---------------------------------------------------------------------------
# The ZED component blocks its grab loop on "Waiting for valid static
# transformations..." until the camera's own TF chain exists. Without an
# accompanying robot_state_publisher the node starts, advertises every topic,
# and publishes nothing at all — with no error logged.

CAMERA_LAUNCH = REPO / "ros2_ws/src/soccer_bringup/launch/camera.launch.py"


def check_camera_launch_has_rsp() -> None:
    if not CAMERA_LAUNCH.exists():
        bad("camera-launch-rsp", f"{CAMERA_LAUNCH} is missing", "restore the file")
        return

    tree = ast.parse(CAMERA_LAUNCH.read_text())
    found = False
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        fname = getattr(func, "id", None) or getattr(func, "attr", None)
        if fname != "Node":
            continue
        for kw in node.keywords:
            if (
                kw.arg == "package"
                and isinstance(kw.value, ast.Constant)
                and kw.value.value == "robot_state_publisher"
            ):
                found = True

    if found:
        ok("camera.launch.py starts a robot_state_publisher for the ZED URDF")
    else:
        bad(
            "camera-launch-rsp",
            "camera.launch.py does not construct a robot_state_publisher Node. "
            "The ZED component will stall forever on static transforms and "
            "publish no images, without logging an error.",
            "add the zed_state_publisher Node back (see docs/TROUBLESHOOTING.md §2).",
        )


# ---------------------------------------------------------------------------
# 3. provision.yml must pin the CDI spec outside tmpfs.
# ---------------------------------------------------------------------------
# The vendor unit writes the spec to /var/run/cdi (tmpfs) and regenerates it at
# boot before the GPU driver is up. The regeneration fails, the tmpfs copy is
# already gone, and then EVERY container fails — not just GPU ones, because
# daemon.json sets default-runtime=nvidia.

PROVISION = REPO / "deploy/ansible/provision.yml"


def check_provision_pins_cdi_path() -> None:
    if not PROVISION.exists():
        bad("cdi-persistence", f"{PROVISION} is missing", "restore the playbook")
        return
    text = PROVISION.read_text()
    if "NVIDIA_CTK_CDI_OUTPUT_FILE_PATH" in text:
        ok("provision.yml pins NVIDIA_CTK_CDI_OUTPUT_FILE_PATH outside tmpfs")
    else:
        bad(
            "cdi-persistence",
            "provision.yml does not set NVIDIA_CTK_CDI_OUTPUT_FILE_PATH. The CDI "
            "spec will be written to tmpfs and lost on every reboot.",
            "restore the 'Persist the CDI spec outside tmpfs' task.",
        )


# ---------------------------------------------------------------------------
# 4. Compose build targets must exist as stages in the referenced Dockerfile.
# ---------------------------------------------------------------------------
# Renaming a stage without updating compose produces a confusing build failure
# far from the cause.

COMPOSE_DIR = REPO / "deploy/compose"


def check_compose_targets_exist() -> None:
    for compose in sorted(COMPOSE_DIR.glob("*.compose.yaml")):
        text = compose.read_text()
        # Intentionally line-based: a YAML parser would need a dependency, and
        # the build blocks in these files are flat.
        dockerfile = None
        for m in re.finditer(
            r"dockerfile:\s*(\S+)|target:\s*(\S+)", text
        ):
            if m.group(1):
                dockerfile = m.group(1)
            elif m.group(2) and dockerfile:
                target = m.group(2)
                path = REPO / dockerfile
                if not path.exists():
                    bad(
                        "compose-target",
                        f"{compose.name} references {dockerfile}, which does not exist",
                        "fix the dockerfile: path",
                    )
                    continue
                stages = {
                    mm.group(2)
                    for ln in path.read_text().splitlines()
                    if (mm := STAGE_RE.match(ln)) and mm.group(2)
                }
                if target in stages:
                    ok(f"{compose.name}: target '{target}' exists in {dockerfile}")
                else:
                    bad(
                        "compose-target",
                        f"{compose.name} builds target '{target}', which is not a "
                        f"stage in {dockerfile}. Known stages: {sorted(stages)}",
                        "rename the target or add the stage",
                    )


# ---------------------------------------------------------------------------
# 5. Markdown relative links must resolve.
# ---------------------------------------------------------------------------
# Deleting or renaming a document silently breaks every link to it. Nothing in a
# normal build reads markdown, so these rot indefinitely.

LINK_RE = re.compile(r"\[[^\]]*\]\(([^)]+)\)")


def check_markdown_links() -> None:
    docs = [
        p
        for p in REPO.rglob("*.md")
        if "ros2_ws/build" not in str(p) and "ros2_ws/install" not in str(p)
    ]
    broken = 0
    for doc in sorted(docs):
        for target in LINK_RE.findall(doc.read_text()):
            if target.startswith(("http://", "https://", "#", "mailto:")):
                continue
            path = target.split("#", 1)[0]
            if not path:
                continue
            if not (doc.parent / path).resolve().exists():
                bad(
                    "markdown-links",
                    f"{doc.relative_to(REPO)} links to '{target}', which does not exist",
                    "fix or remove the link",
                )
                broken += 1
    if broken == 0:
        ok(f"all relative links in {len(docs)} markdown files resolve")


# ---------------------------------------------------------------------------
# 6. Perception must not hardcode camera intrinsics.
# ---------------------------------------------------------------------------
# fieldline_node and projection_node both built their ground-projection model
# from fx=550, cx=320, cy=240 -- a 640x480 camera. The ZED Mini is 1280x720 with
# fx=732.9. Ground projections were wrong by 2.5-8.5 m, and pixels ABOVE THE
# HORIZON were turned into confident ground points. projection_node made it
# worse by subscribing to "camera_info" while the camera published on
# "camera/camera_info", so its placeholders were never overwritten.
#
# Invariant: no source under ros2_ws/src assigns a numeric literal to an
# intrinsic. They come from camera_info or they do not exist.

INTRINSIC_RE = re.compile(r"\b(fx|fy|cx|cy)\s*(?::\s*\w+\s*)?=\s*[0-9]")


def check_no_hardcoded_intrinsics() -> None:
    srcs = [
        p for p in (REPO / "ros2_ws" / "src").rglob("*")
        if p.suffix in {".py", ".cpp", ".hpp"}
        and "/build/" not in str(p) and "/install/" not in str(p)
        # Tests assert ON these values (that a VGA guess misprojects by metres),
        # so they are the one place the constants legitimately appear.
        and "/test/" not in str(p) and not p.name.startswith("test_")
    ]
    hits = 0
    for src in sorted(srcs):
        for n, line in enumerate(src.read_text().splitlines(), 1):
            if line.lstrip().startswith(("#", "//", "*")):
                continue
            if INTRINSIC_RE.search(line):
                # Zeroed placeholders are the sanctioned "not yet known" state.
                if re.search(r"\b(fx|fy|cx|cy)\s*=\s*0(\.0)?\b", line):
                    continue
                bad(
                    "hardcoded-intrinsics",
                    f"{src.relative_to(REPO)}:{n} assigns a camera intrinsic from a "
                    f"literal: {line.strip()}",
                    "take fx/fy/cx/cy from the camera_info topic and refuse to "
                    "project until it arrives.",
                )
                hits += 1
    if hits == 0:
        ok(f"no hardcoded camera intrinsics in {len(srcs)} perception sources")


# ---------------------------------------------------------------------------
# 7. camera_info consumers and publishers must agree on the topic name.
# ---------------------------------------------------------------------------
# ROS convention puts camera_info NEXT TO the image. Subscribing to bare
# "camera_info" silently connects to nothing.

CAMINFO_RE = re.compile(r"""["'](/?[\w/~]*camera_info)["']""")
REMAP_RE = re.compile(r"""\(\s*["'][^"']+["']\s*,\s*["']([^"']+)["']\s*\)""")


def check_camera_info_topic_agrees() -> None:
    srcs = [
        p for p in (REPO / "ros2_ws" / "src").rglob("*")
        if p.suffix in {".py", ".cpp", ".hpp", ".yaml"}
        and "/build/" not in str(p) and "/install/" not in str(p)
    ]
    srcs += [REPO / "deploy" / "compose" / "zed_params_override.yaml"]
    wrong = 0
    for src in sorted(srcs):
        if not src.exists():
            continue
        for n, line in enumerate(src.read_text().splitlines(), 1):
            if line.lstrip().startswith(("#", "//", "*")):
                continue
            # In a remap tuple only the TARGET must follow our contract; the
            # source is the driver's own native topic name.
            remap = REMAP_RE.search(line)
            topics = [remap.group(1)] if remap else CAMINFO_RE.findall(line)
            for topic in topics:
                if "camera_info" not in topic:
                    continue
                if topic.rstrip("/").split("/")[-2:-1] == ["camera"]:
                    continue  # .../camera/camera_info -- correct
                bad(
                    "camera-info-topic",
                    f"{src.relative_to(REPO)}:{n} refers to camera_info as "
                    f"'{topic}', but it is published at 'camera/camera_info'.",
                    "use 'camera/camera_info'; a bare 'camera_info' subscribes "
                    "to a topic with no publishers and never reports an error.",
                )
                wrong += 1
    if wrong == 0:
        ok("all camera_info references use the 'camera/camera_info' contract")


# ---------------------------------------------------------------------------
# 8. The TensorRT headers must be pinned to the installed runtime version.
# ---------------------------------------------------------------------------
# Engines are version-locked to the runtime that built them. The default apt
# candidate for libnvinfer-headers-dev is 11.x, which cannot load our 10.16.2
# engines, so the version must be pinned explicitly and must match libnvinfer10.

RUNTIME_RE = re.compile(r"libnvinfer10[=\s]")
HEADERS_PIN_RE = re.compile(r"libnvinfer-headers-dev=([\d.]+)-")


def check_tensorrt_headers_pinned() -> None:
    dockerfile = REPO / "deploy" / "docker" / "Dockerfile.jetson"
    if not dockerfile.exists():
        return
    text = dockerfile.read_text()
    if "libnvinfer-headers-dev" not in text:
        return
    pins = HEADERS_PIN_RE.findall(text)
    if not pins:
        bad(
            "tensorrt-header-pin",
            "Dockerfile.jetson installs libnvinfer-headers-dev without a version.",
            "pin it (libnvinfer-headers-dev=<ver>) to the same version as "
            "libnvinfer10; the default apt candidate is a major version ahead "
            "and cannot load our engines.",
        )
        return
    ok(f"Dockerfile.jetson pins libnvinfer-headers-dev to {pins[0]}")


def main() -> int:
    check_dockerfile_runtime_deps()
    check_camera_launch_has_rsp()
    check_provision_pins_cdi_path()
    check_compose_targets_exist()
    check_markdown_links()
    check_no_hardcoded_intrinsics()
    check_camera_info_topic_agrees()
    check_tensorrt_headers_pinned()

    for msg in passes:
        print(f"  ok   {msg}")
    if failures:
        print()
        for msg in failures:
            print(f"  FAIL {msg}\n")
        print(f"{len(failures)} invariant(s) violated.")
        return 1
    print(f"\nAll {len(passes)} repository invariants hold.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

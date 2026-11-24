"""robot_model.yaml (+ contact overlay) -> URDF, MJCF, meshes, safety_manifest.yaml.

Each robot's description is written into the humanoid_description package as it installs under
share/humanoid_description/robots/<robot>/ (urdf/, mjcf/, meshes/, config/), and every path one of
its files names is a function of the robot's name alone, never of where the model's sources live.

The emitters live by output: urdf.py, mjcf.py (robot.xml and scene.xml) and configs.py (the safety
manifest, the STM32 wiring and the firmware config).
"""
import os
import shutil

import yaml

from .configs import emit_firmware_config, emit_safety_manifest, emit_stm32_wiring
from .mjcf import emit_mjcf, emit_scene
from .urdf import DESCRIPTION_PACKAGE, emit_urdf

def load(src, overlay_path):
    with open(src) as f:
        model = yaml.safe_load(f)
    overlay = None
    if overlay_path and os.path.exists(overlay_path):
        with open(overlay_path) as f:
            overlay = yaml.safe_load(f)
    return model, overlay

def generate(robot, src, overlay_path, motor_limits, description_dir, export_dir, meshes_dir,
             urdf, mjcf, license_path=None, firmware_config=False, firmware_setup=None,
             stm32_wiring=False):
    """Write robot `robot`'s description to description_dir, laid out as humanoid_description
    installs it under share/humanoid_description/robots/<robot>/, and the files other repositories
    import (the firmware config) to export_dir. Both directories hold nothing else, so `check` can
    regenerate them anywhere and compare the trees."""
    model, overlay = load(src, overlay_path)
    written = []

    def out(*parts):
        written.append("/".join(parts))
        path = os.path.join(description_dir, *parts)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return path

    meshes = set()
    if urdf:
        meshes |= emit_urdf(model, out("urdf", "robot.urdf"), meshes_dir,
                            f"package://{DESCRIPTION_PACKAGE}/robots/{robot}/meshes",
                            f"robots/{robot}/config/safety_manifest.yaml")
    if mjcf:
        home_qpos, mjcf_meshes = emit_mjcf(model, overlay, out("mjcf", "robot.xml"), meshes_dir)
        meshes |= mjcf_meshes
        emit_scene(model, out("mjcf", "scene.xml"), home_qpos)
    # Only the meshes the description names, copied byte for byte: the package must stand alone
    # once installed, and a symlink would dangle there.
    for f in sorted(meshes):
        shutil.copyfile(os.path.join(meshes_dir, f), out("meshes", f))
    # The license the robot's sources came under travels with what is derived from them.
    if license_path:
        shutil.copyfile(license_path, out("LICENSE"))

    emit_safety_manifest(model, motor_limits, out("config", "safety_manifest.yaml"))
    if stm32_wiring:
        emit_stm32_wiring(model, out("config", "stm32_wiring.yaml"))

    if firmware_config:
        os.makedirs(export_dir, exist_ok=True)
        setup = firmware_setup or model["robot"]["name"]
        emit_firmware_config(model, motor_limits, os.path.join(export_dir, "firmware_config.yaml"),
                             setup)
        print(f"generated firmware_config.yaml (+ .sha256) in {export_dir}")

    print(f"generated {len(written)} files in {description_dir}")

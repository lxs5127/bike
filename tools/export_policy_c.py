#!/usr/bin/env python3
"""Export a trusted project NPZ actor to portable float32 C arrays (numpy only)."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def literal(value):
    text = format(float(np.float32(value)), ".9g")
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text + "f"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    with np.load(args.source, allow_pickle=False) as archive:
        metadata = json.loads(str(archive["metadata"]))
        arrays = {k: archive[k].copy() for k in archive.files if k != "metadata"}
    if metadata["observation_size"] != 4 or metadata["policy_memory"] is not True:
        raise ValueError("Expected 4 observations plus internal rotation memory")
    config = metadata["config"]
    if config.get("flywheel_unload_kw", 0) or config.get("flywheel_momentum_ki", 0):
        raise ValueError("External unloading/momentum controllers must be ported first")
    if not 0 < config["action_smoothing_alpha"] <= 1:
        raise ValueError("Invalid action smoothing")
    checkpoint = Path(metadata["checkpoint"])
    if checkpoint.exists() and hashlib.sha256(checkpoint.read_bytes()).hexdigest() != metadata["sha256"]:
        raise ValueError("NPZ metadata does not match the current checkpoint")
    urdf = ET.parse(args.urdf).getroot()
    inertia = float(urdf.find("./link[@name='balance_flywheel']/inertial/inertia").get("ixx"))
    torque = float(urdf.find("./joint[@name='flywheel_joint']/limit").get("effort"))
    specs = [
        ("feature.0.weight", "policy_fc1_weight", (128, 5)),
        ("feature.0.bias", "policy_fc1_bias", (128,)),
        ("feature.2.weight", "policy_fc2_weight", (128, 128)),
        ("feature.2.bias", "policy_fc2_bias", (128,)),
        ("actor_mean.weight", "policy_actor_weight", (1, 128)),
        ("actor_mean.bias", "policy_actor_bias", (1,)),
    ]
    for key, _, shape in specs:
        if arrays[key].shape != shape or not np.isfinite(arrays[key]).all():
            raise ValueError("Invalid weights: " + key)
    header = ["/* Generated; do not edit weights by hand. */", "#ifndef POLICY_WEIGHTS_H",
              "#define POLICY_WEIGHTS_H", "#define POLICY_INPUT_SIZE 5U",
              "#define POLICY_HIDDEN_SIZE 128U",
              "#define POLICY_SIM_TORQUE_LIMIT_NM " + literal(torque),
              "#define POLICY_SIM_SPEED_LIMIT_RAD_S " + literal(config["max_flywheel_speed"]),
              "#define POLICY_SIM_FALL_ANGLE_RAD " + literal(np.deg2rad(config["fall_angle_degrees"])),
              "#define POLICY_SIM_FLYWHEEL_INERTIA " + literal(inertia),
              "#define POLICY_ACTION_ALPHA " + literal(config["action_smoothing_alpha"]), ""]
    source = ['#include "policy_weights.h"', ""]
    for key, symbol, _ in specs:
        values = arrays[key].astype(np.float32).ravel()
        header.append(f"extern const float {symbol}[{values.size}];")
        source.append(f"const float {symbol}[{values.size}] = {{")
        for i in range(0, values.size, 8):
            source.append("    " + ", ".join(literal(v) for v in values[i:i+8]) + ",")
        source.extend(["};", ""])
    header.extend(["", "#endif", ""])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "policy_weights.h").write_text("\n".join(header))
    (args.output_dir / "policy_weights.c").write_text("\n".join(source))
    manifest = dict(source_npz=str(args.source.resolve()),
        npz_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
        checkpoint_sha256=metadata["sha256"],
        urdf_sha256=hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
        network=[5, 128, 128, 1], activation="ELU", mirrored_branches=2,
        weights_float32_bytes=sum(arrays[k].size * 4 for k, _, _ in specs), control_hz=60,
        observations=["sin(roll)", "cos(roll)", "clip(body_roll_rate_rad_s,-5,5)",
                      "clip(flywheel_rad_s/420,-1.5,1.5)"],
        internal_memory="trapezoidal speed integral; clip(rotation_rad/1000,-10,10)",
        simulation_limits_only=dict(torque_nm=torque, speed_rad_s=config["max_flywheel_speed"],
                                    fall_angle_deg=config["fall_angle_degrees"], inertia_kg_m2=inertia),
        config=config)
    (args.output_dir / "policy_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Exported actor: {manifest['weights_float32_bytes']} bytes, mirrored branches, 60 Hz")


if __name__ == "__main__":
    main()

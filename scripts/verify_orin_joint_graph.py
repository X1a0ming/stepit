#!/usr/bin/env python3
"""Verify V12 actor identity and Orin deployment profile contracts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


EXPECTED_VARIANTS = ("v12b", "v12c")
EXPECTED_PROFILES = {
    "go2_motor": {"robot": "go2", "dof": 12, "robot_id": 4, "slice": [3, 15]},
    "b2w_motor": {"robot": "b2w", "dof": 16, "robot_id": 2, "slice": [3, 19]},
}


def fail(message: str) -> None:
    raise RuntimeError(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        fail(f"{path} must contain a JSON object")
    return value


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except ModuleNotFoundError as error:
        fail("PyYAML is required (Ubuntu package: python3-yaml)")
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        fail(f"{path} must contain a YAML mapping")
    return value


def verify_actor(actor_dir: Path, variant: str) -> None:
    actor = actor_dir / "actor.onnx"
    manifest = load_json(actor_dir / "manifest.json")
    if sha256(actor) != manifest.get("actor_sha256"):
        fail(f"{variant} actor hash does not match its manifest")
    if manifest.get("observation_dim") != 458 or manifest.get("action_dim") != 32:
        fail(f"{variant} manifest must declare 458 -> 32")
    if manifest.get("control_rate_hz") != 50:
        fail(f"{variant} control rate must be 50 Hz")
    if set(manifest.get("supported_profiles", ())) != {"go2_motor", "b2w_motor", "cmd_vel"}:
        fail(f"{variant} supported profile set is incomplete")
    actor_config = load_yaml(actor_dir / "actor.yml")
    inputs = actor_config.get("input_fields", {}).get("observation", {})
    outputs = actor_config.get("output_fields", {}).get("action", [])
    if inputs != {"name": "joint_graph_observation", "size": 458}:
        fail(f"{variant} actor.yml input contract is not 458-D joint graph")
    if outputs != [
        {"name": "joint_graph_cmd_vel", "size": 3},
        {"name": "joint_graph_motor", "size": 29},
    ]:
        fail(f"{variant} actor.yml output split is not 3 + 29")
    recurrent = bool(actor_config.get("recurrent_params"))
    if recurrent != bool(manifest.get("recurrent")):
        fail(f"{variant} recurrent declaration mismatch")

    try:
        import onnx
    except ModuleNotFoundError:
        return
    graph = onnx.load(str(actor)).graph
    ordinary_inputs = {item.name: [axis.dim_value for axis in item.type.tensor_type.shape.dim] for item in graph.input}
    ordinary_outputs = {item.name: [axis.dim_value for axis in item.type.tensor_type.shape.dim] for item in graph.output}
    if ordinary_inputs.get("observation", [])[-1:] != [458]:
        fail(f"{variant} ONNX observation input is not 458")
    if ordinary_outputs.get("action", [])[-1:] != [32]:
        fail(f"{variant} ONNX action output is not 32")
    if variant == "v12b" and (
        ordinary_inputs.get("memory_in__level2__hidden") != [1, 128]
        or ordinary_outputs.get("memory_out__level2__hidden") != [1, 128]
    ):
        fail("v12b recurrent hidden-state graph contract is invalid")


def verify_profile(profile_dir: Path, name: str) -> None:
    expected = EXPECTED_PROFILES[name]
    profile = load_json(profile_dir / "profile.json")
    if profile.get("robot") != expected["robot"] or profile.get("dof") != expected["dof"]:
        fail(f"{name} robot/DoF mismatch")
    if profile.get("robot_id") != expected["robot_id"] or profile.get("actor_motor_slice") != expected["slice"]:
        fail(f"{name} robot ID or actor slice mismatch")
    dof = expected["dof"]
    if profile.get("dds_motor_indices") != list(range(dof)):
        fail(f"{name} DDS mapping must be an explicit identity permutation")

    observation = load_yaml(profile_dir / "joint_graph_observation.yml")
    if observation.get("joint_count") != dof or observation.get("robot_id") != expected["robot_id"]:
        fail(f"{name} observation identity mismatch")
    if observation.get("joint_indices") != list(range(dof)):
        fail(f"{name} joint_indices mismatch")
    if len(observation.get("parent_indices", ())) != dof or len(observation.get("control_modes", ())) != dof:
        fail(f"{name} topology/mode length mismatch")
    features = observation.get("joint_features", {})
    required_features = (
        "default_position", "default_velocity", "lower_limit", "upper_limit",
        "velocity_limit", "effort_limit", "stiffness", "damping", "action_scale",
    )
    for feature in required_features:
        if len(features.get(feature, ())) != dof:
            fail(f"{name} {feature} length mismatch")

    motor = load_yaml(profile_dir / "joint_graph_motor_action.yml")
    operator = motor.get("operators", [{}])[0]
    if operator.get("start") != 0 or operator.get("end") != dof:
        fail(f"{name} motor action slice mismatch")
    path = load_yaml(profile_dir / "joint_graph_path_subscriber.yml").get("path_points_body", {})
    if path.get("size") != 40 or path.get("timeout_threshold", 99) > 0.25:
        fail(f"{name} waypoint size/watchdog mismatch")
    expected_layout = {"labels": ["point", "xy"], "sizes": [20, 2], "strides": [40, 2], "data_offset": 0}
    if path.get("expected_layout") != expected_layout:
        fail(f"{name} waypoint layout mismatch")


def verify_installed(repo: Path, workspace: Path) -> None:
    for variant in EXPECTED_VARIANTS:
        expected_hash = load_json(repo / "deploy/orin/actors" / variant / "manifest.json")["actor_sha256"]
        for profile in EXPECTED_PROFILES:
            bundle = workspace / "models" / f"mirlab_joint_graph_{variant}_{profile}"
            if not bundle.is_dir():
                fail(f"installed bundle missing: {bundle}")
            policy = load_yaml(bundle / "policy.yml")
            if policy.get("tailored") != EXPECTED_PROFILES[profile]["robot"]:
                fail(f"{bundle} is tailored to the wrong robot")
            policy_text = (bundle / "policy.yml").read_text(encoding="utf-8")
            if "@POLICY_NAME@" in policy_text or "@VARIANT@" in policy_text:
                fail(f"{bundle}/policy.yml still contains a template placeholder")
            if sha256(bundle / "actor.onnx") != expected_hash:
                fail(f"{bundle}/actor.onnx hash mismatch")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--workspace", type=Path)
    args = parser.parse_args()
    repo = args.repo.resolve()
    for variant in EXPECTED_VARIANTS:
        verify_actor(repo / "deploy/orin/actors" / variant, variant)
    for profile in EXPECTED_PROFILES:
        verify_profile(repo / "deploy/orin/profiles" / profile, profile)
    if args.workspace:
        verify_installed(repo, args.workspace.resolve())
    print("valid Orin deployment: V12B/V12C, Go2 motor, B2W motor, cmd_vel")


if __name__ == "__main__":
    main()

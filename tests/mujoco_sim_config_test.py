#!/usr/bin/env python3

import json
import tempfile
from pathlib import Path

from interface.robot.simulation.mujoco_sim_config import (
    CANONICAL_JOINT_ORDER,
    CROUCH_JOINT_POS,
    DEFAULT_INITIAL_POSE,
    STARTUP_HOLD_KD,
    STARTUP_HOLD_KP,
    initial_base_height,
    initial_joint_pose,
    load_policy_sim_defaults,
)


def expect(condition, message):
    if not condition:
        raise RuntimeError(message)


def write_metadata(root: Path):
    policy_dir = root / "policy" / "ppo"
    policy_dir.mkdir(parents=True)
    metadata = {
        "obs_dim": 48,
        "action_dim": 12,
        "action_semantics": "target_joint_position",
        "joint_order": CANONICAL_JOINT_ORDER,
        "default_joint_pos": [0.0, -0.8, 1.6] * 4,
        "action_scale": [0.25] * 12,
        "lin_vel_scale": 2.0,
        "omega_scale": 0.25,
        "dof_vel_scale": 0.05,
        "target_base_height": 0.28,
        "decimation": 12,
        "policy_frequency_hz": 50.0,
        "pd_update_frequency_hz": 1000.0,
    }
    (policy_dir / "policy_metadata.json").write_text(json.dumps(metadata), encoding="utf-8")


def test_policy_stand_pose_defaults():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        write_metadata(root)
        defaults = load_policy_sim_defaults(root)

        expect(defaults.default_joint_pos == [0.0, -0.8, 1.6] * 4, "default joint pose")
        expect(abs(defaults.target_base_height - 0.28) < 1e-6, "target base height")
        expect(initial_joint_pose("stand", defaults) == defaults.default_joint_pos, "stand joint pose")
        expect(abs(initial_base_height("stand", defaults) - 0.28) < 1e-6, "stand height")


def test_crouch_pose_is_available_for_manual_debugging():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        write_metadata(root)
        defaults = load_policy_sim_defaults(root)

        expect(initial_joint_pose("crouch", defaults) == CROUCH_JOINT_POS, "crouch joint pose")
        expect(abs(initial_base_height("crouch", defaults) - 0.10) < 1e-6, "crouch height")


def test_default_initial_pose_is_crouch_waiting_pose():
    expect(DEFAULT_INITIAL_POSE == "crouch", "default initial pose")


def test_unknown_pose_is_rejected():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        write_metadata(root)
        defaults = load_policy_sim_defaults(root)

        rejected = False
        try:
            initial_joint_pose("sideways", defaults)
        except ValueError:
            rejected = True
        expect(rejected, "unknown pose should fail")


def test_incomplete_metadata_is_rejected():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        policy_dir = root / "policy" / "ppo"
        policy_dir.mkdir(parents=True)
        metadata = {
            "default_joint_pos": [0.0, -0.8, 1.6] * 4,
            "target_base_height": 0.28,
        }
        (policy_dir / "policy_metadata.json").write_text(json.dumps(metadata), encoding="utf-8")

        rejected = False
        try:
            load_policy_sim_defaults(root)
        except ValueError:
            rejected = True
        expect(rejected, "incomplete policy metadata should fail")


def test_wrong_joint_order_is_rejected():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        write_metadata(root)
        metadata_path = root / "policy" / "ppo" / "policy_metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["joint_order"] = list(reversed(CANONICAL_JOINT_ORDER))
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

        rejected = False
        try:
            load_policy_sim_defaults(root)
        except ValueError:
            rejected = True
        expect(rejected, "wrong joint order should fail")


def test_startup_hold_gains_are_enabled():
    expect(STARTUP_HOLD_KP > 0.0, "startup hold kp")
    expect(STARTUP_HOLD_KD > 0.0, "startup hold kd")


if __name__ == "__main__":
    test_policy_stand_pose_defaults()
    test_crouch_pose_is_available_for_manual_debugging()
    test_default_initial_pose_is_crouch_waiting_pose()
    test_unknown_pose_is_rejected()
    test_incomplete_metadata_is_rejected()
    test_wrong_joint_order_is_rejected()
    test_startup_hold_gains_are_enabled()

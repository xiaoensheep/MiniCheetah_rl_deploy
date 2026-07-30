#!/usr/bin/env python3

import json
import tempfile
from pathlib import Path

from interface.robot.simulation.mujoco_sim_config import (
    CANONICAL_JOINT_ORDER,
    CROUCH_HOLD_JOINT_POS,
    CROUCH_JOINT_POS,
    CROUCH_BASE_POS,
    CROUCH_BASE_QUAT,
    CROUCH_QVEL,
    DEFAULT_INITIAL_POSE,
    STARTUP_HOLD_KD,
    STARTUP_HOLD_KP,
    initial_base_pos,
    initial_base_quat,
    initial_base_height,
    initial_joint_pose,
    initial_hold_joint_pose,
    initial_qvel,
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
        "default_joint_pos": [0.0, 0.8, -1.6] * 4,
        "robot_default_joint_pos": [0.0, -0.8, 1.6] * 4,
        "action_scale": [0.25] * 12,
        "lin_vel_scale": 1.0,
        "omega_scale": 1.0,
        "dof_vel_scale": 1.0,
        "target_base_height": 0.28,
        "decimation": 20,
        "policy_frequency_hz": 50.0,
        "pd_update_frequency_hz": 1000.0,
    }
    (policy_dir / "policy_metadata.json").write_text(json.dumps(metadata), encoding="utf-8")


def test_policy_stand_pose_defaults():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        write_metadata(root)
        defaults = load_policy_sim_defaults(root)

        policy_default = [0.0, 0.8, -1.6] * 4
        robot_default = [0.0, -0.8, 1.6] * 4
        expect(defaults.default_joint_pos == policy_default, "policy default joint pose")
        expect(defaults.robot_default_joint_pos == robot_default, "robot default joint pose")
        expect(abs(defaults.target_base_height - 0.28) < 1e-6, "target base height")
        expect(initial_joint_pose("stand", defaults) == robot_default, "stand joint pose")
        expect(abs(initial_base_height("stand", defaults) - 0.28) < 1e-6, "stand height")


def test_crouch_pose_is_available_for_manual_debugging():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        write_metadata(root)
        defaults = load_policy_sim_defaults(root)

        expect(initial_joint_pose("crouch", defaults) == CROUCH_JOINT_POS, "crouch joint pose")
        expect(initial_hold_joint_pose("crouch", defaults) == CROUCH_HOLD_JOINT_POS, "crouch hold joint pose")
        expect(initial_base_pos("crouch", defaults) == CROUCH_BASE_POS, "crouch base position")
        expect(initial_base_quat("crouch") == CROUCH_BASE_QUAT, "crouch base quaternion")
        expect(initial_qvel("crouch", 12) == CROUCH_QVEL, "crouch qvel")
        expect(abs(initial_base_height("crouch", defaults) - 0.096633) < 1e-6, "crouch height")


def test_crouch_hold_pose_stays_inside_deployment_limits():
    expect(CROUCH_HOLD_JOINT_POS[2] <= 2.6, "FR crouch hold calf upper limit")
    expect(CROUCH_HOLD_JOINT_POS[5] <= 2.6, "FL crouch hold calf upper limit")
    expect(CROUCH_HOLD_JOINT_POS != CROUCH_JOINT_POS, "raw keyframe and hold pose should be distinct")


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


def test_reordered_policy_joint_order_is_accepted():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        write_metadata(root)
        metadata_path = root / "policy" / "ppo" / "policy_metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["joint_order"] = list(reversed(CANONICAL_JOINT_ORDER))
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

        defaults = load_policy_sim_defaults(root)
        expect(defaults.robot_default_joint_pos == [0.0, -0.8, 1.6] * 4, "reordered policy joint order")


def test_duplicate_joint_order_is_rejected():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        write_metadata(root)
        metadata_path = root / "policy" / "ppo" / "policy_metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["joint_order"][-1] = metadata["joint_order"][0]
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

        rejected = False
        try:
            load_policy_sim_defaults(root)
        except ValueError:
            rejected = True
        expect(rejected, "duplicate joint order should fail")


def test_startup_hold_gains_are_enabled():
    expect(STARTUP_HOLD_KP > 0.0, "startup hold kp")
    expect(STARTUP_HOLD_KD > 0.0, "startup hold kd")


if __name__ == "__main__":
    test_policy_stand_pose_defaults()
    test_crouch_pose_is_available_for_manual_debugging()
    test_crouch_hold_pose_stays_inside_deployment_limits()
    test_default_initial_pose_is_crouch_waiting_pose()
    test_unknown_pose_is_rejected()
    test_incomplete_metadata_is_rejected()
    test_reordered_policy_joint_order_is_accepted()
    test_duplicate_joint_order_is_rejected()
    test_startup_hold_gains_are_enabled()

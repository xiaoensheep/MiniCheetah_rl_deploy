import json
from dataclasses import dataclass
from pathlib import Path


CANONICAL_JOINT_ORDER = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]
CROUCH_JOINT_POS = [
    -0.12319,
    -1.54732,
    2.60066,
    0.116436,
    -1.5563,
    2.61816,
    -0.0559533,
    -1.50758,
    2.46631,
    0.0480561,
    -1.5152,
    2.48509,
]
CROUCH_BASE_POS = [-0.000623329, 0.00176311, 0.096633]
CROUCH_BASE_QUAT = [0.999062, -0.00688532, 0.0427483, 0.000266976]
CROUCH_QVEL = [
    -1.36518e-05,
    -2.74203e-07,
    1.16291e-06,
    2.13922e-06,
    -2.4024e-05,
    6.34784e-05,
    -4.51138e-05,
    5.44215e-05,
    -1.9761e-05,
    0.000155157,
    0.00182489,
    0.000552896,
    9.23184e-05,
    -3.32965e-05,
    6.74106e-05,
    0.000101832,
    0.000103856,
    3.24305e-05,
]
CROUCH_BASE_HEIGHT = CROUCH_BASE_POS[2]
DEFAULT_INITIAL_POSE = "crouch"
STARTUP_HOLD_KP = 40.0
STARTUP_HOLD_KD = 1.0


@dataclass(frozen=True)
class PolicySimDefaults:
    default_joint_pos: list[float]
    target_base_height: float


def project_root_from_sim_file(sim_file: str | Path) -> Path:
    return Path(sim_file).resolve().parents[3]


def load_policy_sim_defaults(project_root: str | Path) -> PolicySimDefaults:
    metadata_path = Path(project_root) / "policy" / "ppo" / "policy_metadata.json"
    with metadata_path.open("r", encoding="utf-8") as metadata_file:
        metadata = json.load(metadata_file)

    required_fields = (
        "obs_dim",
        "action_dim",
        "action_semantics",
        "joint_order",
        "default_joint_pos",
        "action_scale",
        "lin_vel_scale",
        "omega_scale",
        "dof_vel_scale",
        "target_base_height",
        "decimation",
        "policy_frequency_hz",
        "pd_update_frequency_hz",
    )
    missing_fields = [field for field in required_fields if field not in metadata]
    if missing_fields:
        raise ValueError(f"Mini Cheetah policy metadata missing fields: {missing_fields}")
    if int(metadata["action_dim"]) != 12:
        raise ValueError("Mini Cheetah policy action_dim must be 12")
    if metadata["action_semantics"] != "target_joint_position":
        raise ValueError(f"Unsupported Mini Cheetah action semantics: {metadata['action_semantics']}")
    if metadata["joint_order"] != CANONICAL_JOINT_ORDER:
        raise ValueError("Mini Cheetah policy joint_order must match the canonical deployment order")

    default_joint_pos = [float(value) for value in metadata["default_joint_pos"]]
    if len(default_joint_pos) != 12:
        raise ValueError("Mini Cheetah policy default_joint_pos must contain 12 values")
    action_scale = [float(value) for value in metadata["action_scale"]]
    if len(action_scale) != 12:
        raise ValueError("Mini Cheetah policy action_scale must contain 12 values")
    if int(metadata["decimation"]) <= 0:
        raise ValueError("Mini Cheetah policy decimation must be positive")
    if float(metadata["policy_frequency_hz"]) <= 0.0 or float(metadata["pd_update_frequency_hz"]) <= 0.0:
        raise ValueError("Mini Cheetah policy timing values must be positive")

    return PolicySimDefaults(
        default_joint_pos=default_joint_pos,
        target_base_height=float(metadata["target_base_height"]),
    )


def initial_joint_pose(initial_pose: str, defaults: PolicySimDefaults) -> list[float]:
    if initial_pose == "stand":
        return defaults.default_joint_pos
    if initial_pose == "crouch":
        return CROUCH_JOINT_POS
    raise ValueError(f"Unsupported Mini Cheetah initial pose: {initial_pose}")


def initial_base_height(initial_pose: str, defaults: PolicySimDefaults) -> float:
    if initial_pose == "stand":
        return defaults.target_base_height
    if initial_pose == "crouch":
        return CROUCH_BASE_HEIGHT
    raise ValueError(f"Unsupported Mini Cheetah initial pose: {initial_pose}")


def initial_base_pos(initial_pose: str, defaults: PolicySimDefaults) -> list[float]:
    if initial_pose == "stand":
        return [0.0, 0.0, defaults.target_base_height]
    if initial_pose == "crouch":
        return CROUCH_BASE_POS
    raise ValueError(f"Unsupported Mini Cheetah initial pose: {initial_pose}")


def initial_base_quat(initial_pose: str) -> list[float]:
    if initial_pose == "stand":
        return [1.0, 0.0, 0.0, 0.0]
    if initial_pose == "crouch":
        return CROUCH_BASE_QUAT
    raise ValueError(f"Unsupported Mini Cheetah initial pose: {initial_pose}")


def initial_qvel(initial_pose: str, dof_num: int) -> list[float]:
    if initial_pose == "stand":
        return [0.0] * (6 + dof_num)
    if initial_pose == "crouch":
        if len(CROUCH_QVEL) != 6 + dof_num:
            raise ValueError("Mini Cheetah crouch qvel must match MuJoCo velocity dimension")
        return CROUCH_QVEL
    raise ValueError(f"Unsupported Mini Cheetah initial pose: {initial_pose}")

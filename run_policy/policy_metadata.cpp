#include "policy_metadata.h"

#include <fstream>
#include <filesystem>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace {

std::string ReadFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open policy metadata: " + path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string FindValue(const std::string& text, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*([^,}\\n]+)");
    std::smatch match;
    if (!std::regex_search(text, match, pattern)) {
        throw std::runtime_error("Missing policy metadata field: " + key);
    }
    return match[1].str();
}

std::string FindArrayBody(const std::string& text, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]");
    std::smatch match;
    if (!std::regex_search(text, match, pattern)) {
        throw std::runtime_error("Missing policy metadata array: " + key);
    }
    return match[1].str();
}

bool HasValue(const std::string& text, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:");
    return std::regex_search(text, pattern);
}

int LoadInt(const std::string& text, const std::string& key) {
    return std::stoi(FindValue(text, key));
}

float LoadFloat(const std::string& text, const std::string& key) {
    return std::stof(FindValue(text, key));
}

std::string LoadString(const std::string& text, const std::string& key) {
    std::string value = FindValue(text, key);
    const std::regex pattern("\"([^\"]*)\"");
    std::smatch match;
    if (!std::regex_search(value, match, pattern)) {
        throw std::runtime_error("Policy metadata field is not a string: " + key);
    }
    return match[1].str();
}

std::vector<float> LoadFloatArray(const std::string& text, const std::string& key) {
    const std::string body = FindArrayBody(text, key);
    const std::regex number_pattern("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?");
    std::vector<float> values;

    auto begin = std::sregex_iterator(body.begin(), body.end(), number_pattern);
    auto end = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
        values.push_back(std::stof((*it).str()));
    }

    return values;
}

std::vector<std::string> LoadStringArray(const std::string& text, const std::string& key) {
    const std::string body = FindArrayBody(text, key);
    const std::regex string_pattern("\"([^\"]*)\"");
    std::vector<std::string> values;

    auto begin = std::sregex_iterator(body.begin(), body.end(), string_pattern);
    auto end = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
        values.push_back((*it)[1].str());
    }

    return values;
}

std::vector<float> LoadOptionalFloatArray(
    const std::string& text,
    const std::string& key,
    const std::vector<float>& fallback) {
    if (!HasValue(text, key)) {
        return fallback;
    }
    return LoadFloatArray(text, key);
}

float LoadOptionalFloat(const std::string& text, const std::string& key, float fallback) {
    if (!HasValue(text, key)) {
        return fallback;
    }
    return LoadFloat(text, key);
}

void ValidatePolicyMetadata(const PolicyMetadata& metadata) {
    const std::unordered_set<std::string> canonical_joint_names = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

    if (metadata.obs_dim <= 0) {
        throw std::runtime_error("Policy metadata obs_dim must be positive");
    }
    if (metadata.action_dim <= 0) {
        throw std::runtime_error("Policy metadata action_dim must be positive");
    }
    if (metadata.action_semantics != "target_joint_position") {
        throw std::runtime_error("Unsupported action_semantics: " + metadata.action_semantics);
    }
    if (metadata.joint_order.size() != static_cast<size_t>(metadata.action_dim)) {
        throw std::runtime_error("Policy metadata joint_order size must match action_dim");
    }
    const std::unordered_set<std::string> metadata_joint_names(
        metadata.joint_order.begin(), metadata.joint_order.end());
    if (metadata_joint_names != canonical_joint_names) {
        throw std::runtime_error("Policy metadata joint_order must contain the canonical Mini Cheetah joints");
    }
    if (metadata.default_joint_pos.size() != static_cast<size_t>(metadata.action_dim)) {
        throw std::runtime_error("Policy metadata default_joint_pos size must match action_dim");
    }
    if (metadata.action_scale.size() != static_cast<size_t>(metadata.action_dim)) {
        throw std::runtime_error("Policy metadata action_scale size must match action_dim");
    }
    if (metadata.robot_default_joint_pos.size() != static_cast<size_t>(metadata.action_dim)) {
        throw std::runtime_error("Policy metadata robot_default_joint_pos size must match action_dim");
    }
    if (metadata.joint_position_sign.size() != static_cast<size_t>(metadata.action_dim)) {
        throw std::runtime_error("Policy metadata joint_position_sign size must match action_dim");
    }
    if (metadata.kp.size() != static_cast<size_t>(metadata.action_dim)) {
        throw std::runtime_error("Policy metadata kp size must match action_dim");
    }
    if (metadata.kd.size() != static_cast<size_t>(metadata.action_dim)) {
        throw std::runtime_error("Policy metadata kd size must match action_dim");
    }
    if (metadata.action_clip <= 0.0f) {
        throw std::runtime_error("Policy metadata action_clip must be positive");
    }
    if (metadata.decimation <= 0) {
        throw std::runtime_error("Policy metadata decimation must be positive");
    }
    if (metadata.policy_frequency_hz <= 0.0f || metadata.pd_update_frequency_hz <= 0.0f) {
        throw std::runtime_error("Policy metadata timing values must be positive");
    }
}

}  // namespace

PolicyMetadata LoadPolicyMetadata(const std::string& path) {
    const std::string text = ReadFile(path);

    PolicyMetadata metadata;
    metadata.obs_dim = LoadInt(text, "obs_dim");
    metadata.action_dim = LoadInt(text, "action_dim");
    metadata.action_semantics = LoadString(text, "action_semantics");
    metadata.joint_order = LoadStringArray(text, "joint_order");
    metadata.default_joint_pos = LoadFloatArray(text, "default_joint_pos");
    metadata.robot_default_joint_pos = LoadOptionalFloatArray(
        text, "robot_default_joint_pos", metadata.default_joint_pos);
    metadata.joint_position_sign = LoadOptionalFloatArray(
        text, "joint_position_sign", std::vector<float>(metadata.action_dim, 1.0f));
    metadata.action_scale = LoadFloatArray(text, "action_scale");
    metadata.kp = LoadOptionalFloatArray(text, "kp", std::vector<float>(metadata.action_dim, 20.0f));
    metadata.kd = LoadOptionalFloatArray(text, "kd", std::vector<float>(metadata.action_dim, 1.0f));
    metadata.action_clip = LoadOptionalFloat(text, "action_clip", 1.0f);
    metadata.lin_vel_scale = LoadFloat(text, "lin_vel_scale");
    metadata.omega_scale = LoadFloat(text, "omega_scale");
    metadata.dof_vel_scale = LoadFloat(text, "dof_vel_scale");
    metadata.target_base_height = LoadFloat(text, "target_base_height");
    metadata.decimation = LoadInt(text, "decimation");
    metadata.policy_frequency_hz = LoadFloat(text, "policy_frequency_hz");
    metadata.pd_update_frequency_hz = LoadFloat(text, "pd_update_frequency_hz");

    ValidatePolicyMetadata(metadata);

    return metadata;
}

std::string ResolvePolicyMetadataPath() {
    const std::filesystem::path cwd = std::filesystem::current_path();
    const std::filesystem::path from_project_root = cwd / "policy/ppo/policy_metadata.json";
    const std::filesystem::path from_build_dir = cwd / "../policy/ppo/policy_metadata.json";

    if (std::filesystem::exists(from_project_root)) {
        return from_project_root.lexically_normal().string();
    }
    return from_build_dir.lexically_normal().string();
}

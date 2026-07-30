#include "episode_log.h"

#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace {

std::string EscapeJsonString(const std::string& value) {
    std::ostringstream escaped;
    for (unsigned char c : value) {
        switch (c) {
        case '"':
            escaped << "\\\"";
            break;
        case '\\':
            escaped << "\\\\";
            break;
        case '\b':
            escaped << "\\b";
            break;
        case '\f':
            escaped << "\\f";
            break;
        case '\n':
            escaped << "\\n";
            break;
        case '\r':
            escaped << "\\r";
            break;
        case '\t':
            escaped << "\\t";
            break;
        default:
            if (c < 0x20) {
                escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(c) << std::dec << std::setfill(' ');
            } else {
                escaped << static_cast<char>(c);
            }
            break;
        }
    }
    return escaped.str();
}

int HexDigit(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

std::string DecodeJsonString(const std::string& value, const std::string& key) {
    std::string unescaped;
    for (std::size_t i = 0; i < value.size(); ++i) {
        if (value[i] != '\\') {
            unescaped.push_back(value[i]);
            continue;
        }

        if (i + 1 >= value.size()) {
            throw std::runtime_error("Invalid escape in episode log string field: " + key);
        }

        const char escaped = value[++i];
        switch (escaped) {
        case '"':
        case '\\':
        case '/':
            unescaped.push_back(escaped);
            break;
        case 'b':
            unescaped.push_back('\b');
            break;
        case 'f':
            unescaped.push_back('\f');
            break;
        case 'n':
            unescaped.push_back('\n');
            break;
        case 'r':
            unescaped.push_back('\r');
            break;
        case 't':
            unescaped.push_back('\t');
            break;
        case 'u': {
            if (i + 4 >= value.size()) {
                throw std::runtime_error("Incomplete unicode escape in episode log string field: " + key);
            }
            int codepoint = 0;
            for (int digit = 0; digit < 4; ++digit) {
                const int value_digit = HexDigit(value[i + 1 + digit]);
                if (value_digit < 0) {
                    throw std::runtime_error("Invalid unicode escape in episode log string field: " + key);
                }
                codepoint = (codepoint << 4) | value_digit;
            }
            if (codepoint > 0x7f) {
                throw std::runtime_error("Unsupported unicode escape in episode log string field: " + key);
            }
            unescaped.push_back(static_cast<char>(codepoint));
            i += 4;
            break;
        }
        default:
            throw std::runtime_error("Invalid escape in episode log string field: " + key);
        }
    }
    return unescaped;
}

template <typename Vec>
std::string SerializeVector(const Vec& vec) {
    std::ostringstream out;
    out << std::setprecision(9) << "[";
    for (int i = 0; i < vec.size(); ++i) {
        if (i > 0) {
            out << ",";
        }
        out << vec(i);
    }
    out << "]";
    return out.str();
}

std::string FindStringValue(const std::string& line, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*\"((?:\\\\.|[^\"])*)\"");
    std::smatch match;
    if (!std::regex_search(line, match, pattern)) {
        throw std::runtime_error("Missing episode log string field: " + key);
    }
    return DecodeJsonString(match[1].str(), key);
}

double FindDoubleValue(const std::string& line, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?)");
    std::smatch match;
    if (!std::regex_search(line, match, pattern)) {
        throw std::runtime_error("Missing episode log numeric field: " + key);
    }
    return std::stod(match[1].str());
}

bool FindBoolValue(const std::string& line, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*(true|false)");
    std::smatch match;
    if (!std::regex_search(line, match, pattern)) {
        throw std::runtime_error("Missing episode log boolean field: " + key);
    }
    return match[1].str() == "true";
}

std::string FindArrayBody(const std::string& line, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]");
    std::smatch match;
    if (!std::regex_search(line, match, pattern)) {
        throw std::runtime_error("Missing episode log array field: " + key);
    }
    return match[1].str();
}

types::VecXf ParseVectorXf(const std::string& line, const std::string& key) {
    const std::string body = FindArrayBody(line, key);
    const std::regex number_pattern("\\s*([-+]?(?:(?:[0-9]+(?:\\.[0-9]*)?)|(?:\\.[0-9]+))(?:[eE][-+]?[0-9]+)?)\\s*");
    std::vector<float> values;
    std::size_t pos = 0;

    auto skip_ws = [&body](std::size_t* index) {
        while (*index < body.size() &&
               (body[*index] == ' ' || body[*index] == '\t' || body[*index] == '\n' || body[*index] == '\r')) {
            ++(*index);
        }
    };

    skip_ws(&pos);
    while (pos < body.size()) {
        std::smatch match;
        const auto begin = body.cbegin() + static_cast<std::ptrdiff_t>(pos);
        if (!std::regex_search(begin, body.cend(), match, number_pattern, std::regex_constants::match_continuous)) {
            throw std::runtime_error("Malformed episode log numeric array field: " + key);
        }
        values.push_back(std::stof(match[1].str()));
        pos += match.length();

        if (pos == body.size()) {
            break;
        }
        if (body[pos] != ',') {
            throw std::runtime_error("Malformed episode log numeric array field: " + key);
        }
        ++pos;
        skip_ws(&pos);
        if (pos == body.size()) {
            throw std::runtime_error("Malformed episode log numeric array field: " + key);
        }
    }

    types::VecXf vec(values.size());
    for (int i = 0; i < static_cast<int>(values.size()); ++i) {
        vec(i) = values[i];
    }
    return vec;
}

types::Vec3f ParseVec3f(const std::string& line, const std::string& key) {
    const types::VecXf vec = ParseVectorXf(line, key);
    if (vec.size() != 3) {
        throw std::runtime_error("Episode log field must contain 3 values: " + key);
    }
    return types::Vec3f(vec(0), vec(1), vec(2));
}

}  // namespace

std::string SerializeEpisodeLogRecord(const EpisodeLogRecord& record) {
    std::ostringstream out;
    out << std::setprecision(9)
        << "{"
        << "\"timestamp\":" << record.timestamp << ","
        << "\"state_machine_state\":\"" << EscapeJsonString(record.state_machine_state) << "\","
        << "\"user_command\":" << SerializeVector(record.user_command) << ","
        << "\"observation\":" << SerializeVector(record.observation) << ","
        << "\"raw_action\":" << SerializeVector(record.raw_action) << ","
        << "\"clipped_action\":" << SerializeVector(record.clipped_action) << ","
        << "\"target_joint_pos\":" << SerializeVector(record.target_joint_pos) << ","
        << "\"joint_pos\":" << SerializeVector(record.joint_pos) << ","
        << "\"joint_vel\":" << SerializeVector(record.joint_vel) << ","
        << "\"joint_tau\":" << SerializeVector(record.joint_tau) << ","
        << "\"imu_rpy\":" << SerializeVector(record.imu_rpy) << ","
        << "\"imu_omega\":" << SerializeVector(record.imu_omega) << ","
        << "\"imu_acc\":" << SerializeVector(record.imu_acc) << ","
        << "\"base_lin_vel_body\":" << SerializeVector(record.base_lin_vel_body) << ","
        << "\"policy_entry_gate_passed\":" << (record.policy_entry_gate_passed ? "true" : "false") << ","
        << "\"policy_entry_gate_reason\":\"" << EscapeJsonString(record.policy_entry_gate_reason) << "\","
        << "\"clamp_applied\":" << (record.clamp_applied ? "true" : "false") << ","
        << "\"clamp_reason\":\"" << EscapeJsonString(record.clamp_reason) << "\","
        << "\"policy_inference_ms\":" << record.policy_inference_ms << ","
        << "\"control_dt\":" << record.control_dt
        << "}";
    return out.str();
}

EpisodeLogRecord ParseEpisodeLogRecord(const std::string& line) {
    EpisodeLogRecord record;
    record.timestamp = FindDoubleValue(line, "timestamp");
    record.state_machine_state = FindStringValue(line, "state_machine_state");
    record.user_command = ParseVec3f(line, "user_command");
    record.observation = ParseVectorXf(line, "observation");
    record.raw_action = ParseVectorXf(line, "raw_action");
    record.clipped_action = ParseVectorXf(line, "clipped_action");
    record.target_joint_pos = ParseVectorXf(line, "target_joint_pos");
    record.joint_pos = ParseVectorXf(line, "joint_pos");
    record.joint_vel = ParseVectorXf(line, "joint_vel");
    record.joint_tau = ParseVectorXf(line, "joint_tau");
    record.imu_rpy = ParseVec3f(line, "imu_rpy");
    record.imu_omega = ParseVec3f(line, "imu_omega");
    record.imu_acc = ParseVec3f(line, "imu_acc");
    record.base_lin_vel_body = ParseVec3f(line, "base_lin_vel_body");
    record.policy_entry_gate_passed = FindBoolValue(line, "policy_entry_gate_passed");
    record.policy_entry_gate_reason = FindStringValue(line, "policy_entry_gate_reason");
    record.clamp_applied = FindBoolValue(line, "clamp_applied");
    record.clamp_reason = FindStringValue(line, "clamp_reason");
    record.policy_inference_ms = static_cast<float>(FindDoubleValue(line, "policy_inference_ms"));
    record.control_dt = static_cast<float>(FindDoubleValue(line, "control_dt"));
    return record;
}

EpisodeLogger::EpisodeLogger(std::string path) : path_(std::move(path)) {}

void EpisodeLogger::Append(const EpisodeLogRecord& record) const {
    std::ofstream out(path_, std::ios::app);
    if (!out) {
        throw std::runtime_error("Failed to open episode log for append: " + path_);
    }
    out << SerializeEpisodeLogRecord(record) << "\n";
}

std::vector<EpisodeLogRecord> ReadEpisodeLog(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Failed to open episode log for read: " + path);
    }

    std::vector<EpisodeLogRecord> records;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty()) {
            records.push_back(ParseEpisodeLogRecord(line));
        }
    }
    return records;
}

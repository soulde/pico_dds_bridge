#include "pico_dds_bridge/parser.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace pico_dds_bridge {
namespace {

using json = nlohmann::json;

json jsonish(const json& value) {
    if (value.is_object() || value.is_array()) {
        return value;
    }

    if (value.is_string()) {
        const auto& s = value.get_ref<const std::string&>();
        try {
            return json::parse(s);
        } catch (...) {
            return json{};
        }
    }

    return json{};
}

const json* member_any(
    const json& object,
    std::initializer_list<std::string_view> names) {

    if (!object.is_object()) {
        return nullptr;
    }

    for (const auto name : names) {
        const auto it = object.find(std::string(name));
        if (it != object.end()) {
            return &(*it);
        }
    }

    return nullptr;
}

std::int64_t int64_or(
    const json& object,
    std::initializer_list<std::string_view> names,
    const std::int64_t fallback = 0) {

    const json* value = member_any(object, names);
    if (value == nullptr) {
        return fallback;
    }

    try {
        if (value->is_number_integer() || value->is_number_unsigned()) {
            return value->get<std::int64_t>();
        }
        if (value->is_string()) {
            return std::stoll(value->get<std::string>());
        }
    } catch (...) {
    }

    return fallback;
}

std::uint32_t uint32_or(
    const json& object,
    std::initializer_list<std::string_view> names,
    const std::uint32_t fallback = 0) {

    const auto value = int64_or(object, names, static_cast<std::int64_t>(fallback));
    return value < 0 ? fallback : static_cast<std::uint32_t>(value);
}

float float_or(
    const json& object,
    std::initializer_list<std::string_view> names,
    const float fallback = 0.0F) {

    const json* value = member_any(object, names);
    if (value == nullptr) {
        return fallback;
    }

    try {
        if (value->is_number()) {
            return value->get<float>();
        }
        if (value->is_string()) {
            return std::stof(value->get<std::string>());
        }
    } catch (...) {
    }

    return fallback;
}

bool bool_or(
    const json& object,
    std::initializer_list<std::string_view> names,
    const bool fallback = false) {

    const json* value = member_any(object, names);
    if (value == nullptr) {
        return fallback;
    }

    try {
        if (value->is_boolean()) {
            return value->get<bool>();
        }
        if (value->is_number_integer()) {
            return value->get<int>() != 0;
        }
    } catch (...) {
    }

    return fallback;
}

std::vector<double> parse_csv_numbers(const json& value) {
    std::string s;

    if (value.is_string()) {
        s = value.get<std::string>();
    } else {
        return {};
    }

    std::vector<double> result;
    std::stringstream stream(s);
    std::string token;

    while (std::getline(stream, token, ',')) {
        try {
            result.push_back(std::stod(token));
        } catch (...) {
            return {};
        }
    }

    return result;
}

bool parse_pose(const json& object, Pose& out) {
    const json* pose = member_any(object, {"pose", "p"});
    if (pose == nullptr) {
        return false;
    }

    const auto values = parse_csv_numbers(*pose);
    if (values.size() != 7) {
        return false;
    }

    out.position = Vec3{values[0], values[1], values[2]};
    out.orientation = Quaternion{
        values[3], values[4], values[5], values[6]
    };
    return true;
}

void parse_vec6(
    const json& object,
    std::initializer_list<std::string_view> keys,
    Vec3& linear,
    Vec3& angular) {

    const json* value = member_any(object, keys);
    if (value == nullptr) {
        return;
    }

    const auto values = parse_csv_numbers(*value);
    if (values.size() != 6) {
        return;
    }

    linear = Vec3{values[0], values[1], values[2]};
    angular = Vec3{values[3], values[4], values[5]};
}

TrackingState parse_tracking_state(
    const json& object,
    const std::int64_t fallback_timestamp_ns = 0) {

    TrackingState state{};

    state.status = uint32_or(object, {"status", "s"}, 0);
    state.timestamp_ns = int64_or(
        object, {"timeStampNs", "timestampNs"}, fallback_timestamp_ns);

    const bool pose_ok = parse_pose(object, state.pose);

    parse_vec6(
        object, {"va"},
        state.linear_velocity,
        state.angular_velocity);

    parse_vec6(
        object, {"wva"},
        state.linear_acceleration,
        state.angular_acceleration);

    // Head uses status=0/1 in the official examples. Hand joints use a
    // bit-mask. Body/Motion can omit status. A successfully parsed pose is
    // therefore the most portable baseline validity signal.
    state.valid = pose_ok;

    if (const json* status = member_any(object, {"status"}); status != nullptr) {
        state.valid = pose_ok && uint32_or(object, {"status"}, 0) != 0;
    }

    if (const json* hand_status = member_any(object, {"s"}); hand_status != nullptr) {
        const std::uint32_t bits = uint32_or(object, {"s"}, 0);
        state.valid = pose_ok && ((bits & 0x3U) != 0U);
    }

    return state;
}

ControllerState parse_controller(
    const json& object,
    const std::int64_t timestamp_ns) {

    ControllerState result{};
    result.tracking = parse_tracking_state(object, timestamp_ns);

    result.axis_x = float_or(object, {"axisX"});
    result.axis_y = float_or(object, {"axisY"});
    result.grip = float_or(object, {"grip"});
    result.trigger = float_or(object, {"trigger"});

    result.axis_click = bool_or(object, {"axisClick"});
    result.primary_button = bool_or(object, {"primaryButton"});
    result.secondary_button = bool_or(object, {"secondaryButton"});
    result.menu_button = bool_or(object, {"menuButton"});

    return result;
}

HandState parse_hand(const json& object) {
    HandState result{};

    result.active = int64_or(object, {"isActive"}, 0) != 0;
    result.scale = float_or(object, {"scale"}, 1.0F);
    result.timestamp_ns = int64_or(object, {"timeStampNs", "timestampNs"}, 0);

    const json* joints_value = member_any(object, {"HandJointLocations", "joints"});
    if (joints_value == nullptr || !joints_value->is_array()) {
        return result;
    }

    const auto count = std::min<std::size_t>(
        joints_value->size(), result.joints.size());

    result.count = static_cast<std::uint32_t>(count);

    for (std::size_t i = 0; i < count; ++i) {
        const json& joint = (*joints_value)[i];
        result.joints[i].tracking =
            parse_tracking_state(joint, result.timestamp_ns);
        result.joints[i].radius = float_or(joint, {"r"});
    }

    return result;
}

json child_json(
    const json& root,
    std::initializer_list<std::string_view> names) {

    const json* child = member_any(root, names);
    return child == nullptr ? json{} : jsonish(*child);
}

}  // namespace

std::optional<TrackingFrame> TrackingParser::parse(
    const std::string_view callback_json,
    const std::uint64_t sequence,
    const std::int64_t receive_timestamp_ns) const {

    json envelope;

    try {
        envelope = json::parse(callback_json);
    } catch (...) {
        return std::nullopt;
    }

    // XRoboToolkit has used both "functionName" and "function" in examples.
    const json* function_value = member_any(envelope, {"functionName", "function"});
    if (function_value != nullptr && function_value->is_string()) {
        const auto function_name = function_value->get<std::string>();
        if (function_name != "Tracking") {
            return std::nullopt;
        }
    }

    json root = envelope;
    if (const json* value = member_any(envelope, {"value"}); value != nullptr) {
        root = jsonish(*value);
        if (!root.is_object()) {
            return std::nullopt;
        }
    }

    TrackingFrame frame{};
    frame.sequence = sequence;
    frame.receive_timestamp_ns = receive_timestamp_ns;
    frame.source_timestamp_ns =
        int64_or(root, {"timeStampNs", "timestampNs"}, 0);
    frame.input_mode = static_cast<std::int32_t>(
        int64_or(root, {"Input", "input"}, -1));

    // Head
    const json head = child_json(root, {"Head", "head"});
    if (head.is_object()) {
        frame.head = parse_tracking_state(
            head, frame.source_timestamp_ns);
    }

    // Controllers
    const json controllers =
        child_json(root, {"Controller", "controller"});
    if (controllers.is_object()) {
        const json left = child_json(controllers, {"left", "Left"});
        const json right = child_json(controllers, {"right", "Right"});

        const auto ts = int64_or(
            controllers,
            {"timeStampNs", "timestampNs"},
            frame.source_timestamp_ns);

        if (left.is_object()) {
            frame.left_controller = parse_controller(left, ts);
        }
        if (right.is_object()) {
            frame.right_controller = parse_controller(right, ts);
        }
    }

    // Hand tracking
    const json hands = child_json(root, {"Hand", "hand"});
    if (hands.is_object()) {
        const json left = child_json(hands, {"leftHand", "left", "Left"});
        const json right = child_json(hands, {"rightHand", "right", "Right"});

        if (left.is_object()) {
            frame.left_hand = parse_hand(left);
        }
        if (right.is_object()) {
            frame.right_hand = parse_hand(right);
        }
    }

    // Full-body tracking, fixed maximum of 24 official roles.
    const json body = child_json(root, {"Body", "body"});
    if (body.is_object()) {
        const auto body_ts = int64_or(
            body,
            {"timeStampNs", "timestampNs"},
            frame.source_timestamp_ns);

        if (const json* joints = member_any(body, {"joints"});
            joints != nullptr && joints->is_array()) {

            const auto count = std::min<std::size_t>(
                joints->size(), frame.body.size());
            frame.body_count = static_cast<std::uint32_t>(count);

            for (std::size_t i = 0; i < count; ++i) {
                const json& joint = (*joints)[i];
                frame.body[i].role = static_cast<std::uint32_t>(i);
                frame.body[i].tracking =
                    parse_tracking_state(joint, body_ts);
                frame.body[i].imu_timestamp_ns =
                    int64_or(joint, {"t"}, 0);
            }
        }
    }

    // Independent PICO motion trackers. The bridge reserves five slots; tracker_count
    // carries the actual number received from XRoboToolkit.
    const json motion = child_json(root, {"Motion", "motion"});
    if (motion.is_object()) {
        const auto motion_ts = int64_or(
            motion,
            {"timeStampNs", "timestampNs"},
            frame.source_timestamp_ns);

        if (const json* joints = member_any(motion, {"joints"});
            joints != nullptr && joints->is_array()) {

            const auto count = std::min<std::size_t>(
                joints->size(), frame.trackers.size());
            frame.tracker_count = static_cast<std::uint32_t>(count);

            for (std::size_t i = 0; i < count; ++i) {
                frame.trackers[i].index = static_cast<std::uint32_t>(i);
                frame.trackers[i].tracking =
                    parse_tracking_state((*joints)[i], motion_ts);
            }
        }
    }

    return frame;
}

}  // namespace pico_dds_bridge

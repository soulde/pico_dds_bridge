#include "pico_dds_bridge/parser.hpp"

#include <simdjson.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>

namespace pico_dds_bridge {
namespace {

static const std::size_t kParserMaxJsonSize = 256U * 1024U;

bool parse_numbers(const char* text, std::size_t length, double* values, std::size_t count) {
    const char* current = text;
    const char* end = text + length;
    for (std::size_t i = 0; i < count; ++i) {
        while (current < end && (*current == ' ' || *current == '\t')) ++current;
        errno = 0;
        char* next = nullptr;
        values[i] = std::strtod(current, &next);
        if (next == current || errno == ERANGE) return false;
        current = next;
        if (i + 1 < count) {
            if (current >= end || *current != ',') return false;
            ++current;
        }
    }
    return true;
}

simdjson::simdjson_result<simdjson::ondemand::value> field(
    simdjson::ondemand::object& object,
    const char* name) {
    return object.find_field_unordered(name);
}

bool get_text(simdjson::ondemand::value value, const char*& text, std::size_t& length) {
    auto string_result = value.get_string();
    if (string_result.error()) return false;
    text = string_result.value_unsafe().data();
    length = string_result.value_unsafe().size();
    return true;
}

bool get_int64(simdjson::ondemand::object& object, const char* name, std::int64_t& value) {
    auto result = field(object, name).get_int64();
    if (result.error()) return false;
    value = result.value_unsafe();
    return true;
}

bool get_uint32(simdjson::ondemand::object& object, const char* name, std::uint32_t& value) {
    auto result = field(object, name).get_uint64();
    if (result.error() || result.value_unsafe() > 0xffffffffULL) return false;
    value = static_cast<std::uint32_t>(result.value_unsafe());
    return true;
}

bool get_double(simdjson::ondemand::object& object, const char* name, double& value) {
    auto result = field(object, name).get_double();
    if (result.error()) return false;
    value = result.value_unsafe();
    return true;
}

bool get_bool(simdjson::ondemand::object& object, const char* name, bool& value) {
    auto result = field(object, name).get_bool();
    if (!result.error()) {
        value = result.value_unsafe();
        return true;
    }
    std::uint32_t integer = 0;
    if (get_uint32(object, name, integer)) {
        value = integer != 0;
        return true;
    }
    return false;
}

bool get_pose(simdjson::ondemand::object& object, pico_dds_Pose& pose) {
    auto value = field(object, "pose");
    if (value.error() == simdjson::NO_SUCH_FIELD) value = field(object, "p");
    const char* text = nullptr;
    std::size_t length = 0;
    if (value.error() || !get_text(value.value_unsafe(), text, length)) return false;
    double values[7]{};
    if (!parse_numbers(text, length, values, 7)) return false;
    pose.position.x = values[0]; pose.position.y = values[1]; pose.position.z = values[2];
    pose.orientation.x = values[3]; pose.orientation.y = values[4];
    pose.orientation.z = values[5]; pose.orientation.w = values[6];
    return true;
}

void get_vec6(simdjson::ondemand::object& object, const char* name, pico_dds_Vec3& linear, pico_dds_Vec3& angular) {
    auto value = field(object, name);
    const char* text = nullptr;
    std::size_t length = 0;
    if (value.error() || !get_text(value.value_unsafe(), text, length)) return;
    double values[6]{};
    if (!parse_numbers(text, length, values, 6)) return;
    linear.x = values[0]; linear.y = values[1]; linear.z = values[2];
    angular.x = values[3]; angular.y = values[4]; angular.z = values[5];
}

void parse_state_object(simdjson::ondemand::object& object, std::int64_t fallback, pico_dds_TrackingState& output) {
    std::uint32_t status = 0;
    const bool has_status = get_uint32(object, "status", status);
    const bool has_hand_status = !has_status && get_uint32(object, "s", status);
    output.status = status;
    output.timestamp_ns = fallback;
    get_int64(object, "timeStampNs", output.timestamp_ns) || get_int64(object, "timestampNs", output.timestamp_ns);
    const bool pose_ok = get_pose(object, output.pose);
    get_vec6(object, "va", output.linear_velocity, output.angular_velocity);
    get_vec6(object, "wva", output.linear_acceleration, output.angular_acceleration);
    output.valid = pose_ok && (!has_status || (has_hand_status ? (status & 0x3U) != 0U : status != 0));
}

void parse_state(simdjson::ondemand::value value, std::int64_t fallback, pico_dds_TrackingState& output) {
    auto object_result = value.get_object();
    if (!object_result.error()) parse_state_object(object_result.value_unsafe(), fallback, output);
}

void parse_controller(simdjson::ondemand::value value, std::int64_t timestamp, pico_dds_ControllerState& output) {
    auto object_result = value.get_object();
    if (object_result.error()) return;
    simdjson::ondemand::object object = object_result.value_unsafe();
    parse_state_object(object, timestamp, output.tracking);
    double number = 0.0;
    if (get_double(object, "axisX", number)) output.axis_x = static_cast<float>(number);
    if (get_double(object, "axisY", number)) output.axis_y = static_cast<float>(number);
    if (get_double(object, "grip", number)) output.grip = static_cast<float>(number);
    if (get_double(object, "trigger", number)) output.trigger = static_cast<float>(number);
    bool button = false;
    if (get_bool(object, "axisClick", button)) output.axis_click = button;
    if (get_bool(object, "primaryButton", button)) output.primary_button = button;
    if (get_bool(object, "secondaryButton", button)) output.secondary_button = button;
    if (get_bool(object, "menuButton", button)) output.menu_button = button;
}

void parse_hand(simdjson::ondemand::value value, pico_dds_HandState& output) {
    auto object_result = value.get_object();
    if (object_result.error()) return;
    simdjson::ondemand::object object = object_result.value_unsafe();
    std::uint32_t active = 0;
    get_uint32(object, "isActive", active);
    output.active = active != 0;
    double scale = 1.0;
    if (get_double(object, "scale", scale)) output.scale = static_cast<float>(scale);
    get_int64(object, "timeStampNs", output.timestamp_ns) || get_int64(object, "timestampNs", output.timestamp_ns);
    auto joints_value = field(object, "HandJointLocations");
    if (joints_value.error() == simdjson::NO_SUCH_FIELD) joints_value = field(object, "joints");
    if (joints_value.error()) return;
    auto joints_result = joints_value.value_unsafe().get_array();
    if (joints_result.error()) return;
    std::size_t index = 0;
    for (auto joint_result : joints_result.value_unsafe()) {
        if (index >= 26) break;
        if (joint_result.error()) continue;
        simdjson::ondemand::value joint = joint_result.value_unsafe();
        parse_state(joint, output.timestamp_ns, output.joints[index].tracking);
        auto joint_object = joint.get_object();
        if (!joint_object.error()) {
            double radius = 0.0;
            if (get_double(joint_object.value_unsafe(), "r", radius)) output.joints[index].radius = static_cast<float>(radius);
        }
        ++index;
    }
    output.count = static_cast<std::uint32_t>(index);
}

void parse_body(simdjson::ondemand::value value, std::int64_t timestamp, pico_dds_TrackingFrame& output) {
    auto object_result = value.get_object();
    if (object_result.error()) return;
    simdjson::ondemand::object object = object_result.value_unsafe();
    std::int64_t body_timestamp = timestamp;
    get_int64(object, "timeStampNs", body_timestamp) || get_int64(object, "timestampNs", body_timestamp);
    auto joints = field(object, "joints").get_array();
    if (joints.error()) return;
    std::size_t index = 0;
    for (auto joint_result : joints.value_unsafe()) {
        if (index >= 24) break;
        if (joint_result.error()) continue;
        simdjson::ondemand::value joint = joint_result.value_unsafe();
        output.body[index].role = static_cast<std::uint32_t>(index);
        parse_state(joint, body_timestamp, output.body[index].tracking);
        auto joint_object = joint.get_object();
        if (!joint_object.error()) get_int64(joint_object.value_unsafe(), "t", output.body[index].imu_timestamp_ns);
        ++index;
    }
    output.body_count = static_cast<std::uint32_t>(index);
}

void parse_motion(simdjson::ondemand::value value, std::int64_t timestamp, pico_dds_TrackingFrame& output) {
    auto object_result = value.get_object();
    if (object_result.error()) return;
    simdjson::ondemand::object object = object_result.value_unsafe();
    std::int64_t motion_timestamp = timestamp;
    get_int64(object, "timeStampNs", motion_timestamp) || get_int64(object, "timestampNs", motion_timestamp);
    auto joints = field(object, "joints").get_array();
    if (joints.error()) return;
    std::size_t index = 0;
    for (auto joint_result : joints.value_unsafe()) {
        if (index >= 5) break;
        if (joint_result.error()) continue;
        simdjson::ondemand::value joint = joint_result.value_unsafe();
        output.trackers[index].index = static_cast<std::uint32_t>(index);
        parse_state(joint, motion_timestamp, output.trackers[index].tracking);
        ++index;
    }
    output.tracker_count = static_cast<std::uint32_t>(index);
}

}  // namespace

struct TrackingParser::Impl {
    simdjson::ondemand::parser parser;
    simdjson::ondemand::parser inner_parser;
    std::unique_ptr<char[]> inner_buffer;
    Impl() : inner_buffer(new char[kParserMaxJsonSize + simdjson::SIMDJSON_PADDING]) {}
};

TrackingParser::TrackingParser() : impl_(new Impl()) {}
TrackingParser::~TrackingParser() { delete impl_; }

bool TrackingParser::parse_into(const char* json, std::size_t length, std::size_t capacity, std::uint64_t sequence, std::int64_t receive_timestamp_ns, pico_dds_TrackingFrame& output) {
    if (!json || capacity < length + simdjson::SIMDJSON_PADDING) return false;
    auto document_result = impl_->parser.iterate(json, length, capacity);
    if (document_result.error()) return false;
    auto envelope_result = document_result.value_unsafe().get_object();
    if (envelope_result.error()) return false;
    simdjson::ondemand::object envelope = envelope_result.value_unsafe();
    simdjson::ondemand::object root;
    auto value = envelope["value"];
    const char* inner_text = nullptr;
    std::size_t inner_length = 0;
    if (value.error() || !get_text(value.value_unsafe(), inner_text, inner_length)) return false;
    if (inner_length > kParserMaxJsonSize) return false;
    std::memcpy(impl_->inner_buffer.get(), inner_text, inner_length);
    std::memset(impl_->inner_buffer.get() + inner_length, 0, simdjson::SIMDJSON_PADDING);
    auto inner_result = impl_->inner_parser.iterate(impl_->inner_buffer.get(), inner_length, kParserMaxJsonSize + simdjson::SIMDJSON_PADDING);
    if (inner_result.error()) return false;
    auto inner_object = inner_result.value_unsafe().get_object();
    if (inner_object.error()) return false;
    root = inner_object.value_unsafe();

    std::memset(&output, 0, sizeof(output));
    output.frame_seq = sequence;
    output.receive_timestamp_ns = receive_timestamp_ns;
    output.input_mode = -1;

    for (auto root_field : root) {
        auto key_result = root_field.unescaped_key();
        if (key_result.error()) {
            continue;
        }
        const auto key = key_result.value_unsafe();
        if (key == "timeStampNs" || key == "timestampNs") {
            auto timestamp = root_field.value().get_int64();
            if (!timestamp.error()) {
                output.source_timestamp_ns = timestamp.value_unsafe();
            }
        } else if (key == "Input" || key == "input") {
            auto input = root_field.value().get_int64();
            if (!input.error()) {
                output.input_mode = static_cast<std::int32_t>(input.value_unsafe());
            }
        } else if (key == "Head" || key == "head") {
            parse_state(root_field.value(), output.source_timestamp_ns, output.head);
        } else if (key == "Controller" || key == "controller") {
            auto object_result = root_field.value().get_object();
            if (!object_result.error()) {
                auto object = object_result.value_unsafe();
                auto left = field(object, "left");
                if (left.error() == simdjson::NO_SUCH_FIELD) left = field(object, "Left");
                auto right = field(object, "right");
                if (right.error() == simdjson::NO_SUCH_FIELD) right = field(object, "Right");
                if (!left.error()) parse_controller(left.value_unsafe(), output.source_timestamp_ns, output.left_controller);
                if (!right.error()) parse_controller(right.value_unsafe(), output.source_timestamp_ns, output.right_controller);
            }
        } else if (key == "Hand" || key == "hand") {
            auto object_result = root_field.value().get_object();
            if (!object_result.error()) {
                auto object = object_result.value_unsafe();
                auto left = field(object, "leftHand");
                if (left.error() == simdjson::NO_SUCH_FIELD) left = field(object, "left");
                auto right = field(object, "rightHand");
                if (right.error() == simdjson::NO_SUCH_FIELD) right = field(object, "right");
                if (!left.error()) parse_hand(left.value_unsafe(), output.left_hand);
                if (!right.error()) parse_hand(right.value_unsafe(), output.right_hand);
            }
        } else if (key == "Body" || key == "body") {
            parse_body(root_field.value(), output.source_timestamp_ns, output);
        } else if (key == "Motion" || key == "motion") {
            parse_motion(root_field.value(), output.source_timestamp_ns, output);
        }
    }
    return true;
}

}  // namespace pico_dds_bridge
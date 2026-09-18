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

void get_vec6_text(const char* text, std::size_t length, pico_dds_Vec3& linear, pico_dds_Vec3& angular) {
    double values[6]{};
    if (!parse_numbers(text, length, values, 6)) return;
    linear.x = values[0]; linear.y = values[1]; linear.z = values[2];
    angular.x = values[3]; angular.y = values[4]; angular.z = values[5];
}

bool get_pose_text(const char* text, std::size_t length, pico_dds_Pose& pose) {
    double values[7]{};
    if (!parse_numbers(text, length, values, 7)) return false;
    pose.position.x = values[0]; pose.position.y = values[1]; pose.position.z = values[2];
    pose.orientation.x = values[3]; pose.orientation.y = values[4];
    pose.orientation.z = values[5]; pose.orientation.w = values[6];
    return true;
}

// simdjson ondemand is forward-only: unordered field lookups after the object
// has been consumed return errors. Parse every state-like object in a single
// ordered pass over its fields instead. When controller is non-null the same
// pass also consumes the controller axis/button fields.
void parse_state_object(
    simdjson::ondemand::object& object,
    std::int64_t fallback,
    pico_dds_TrackingState& output,
    pico_dds_ControllerState* controller = nullptr,
    float* radius = nullptr,
    std::int64_t* extra_timestamp = nullptr) {
    std::uint32_t status = 0;
    bool has_status = false;
    bool has_hand_status = false;
    bool pose_ok = false;
    output.timestamp_ns = fallback;

    for (auto field_result : object) {
        auto key_result = field_result.unescaped_key();
        if (key_result.error()) continue;
        const auto key = key_result.value_unsafe();
        if (key == "status" || key == "s") {
            const bool hand = key == "s";
            auto number = field_result.value().get_uint64();
            if (!number.error()) {
                if (hand) has_hand_status = true; else has_status = true;
                status = static_cast<std::uint32_t>(number.value_unsafe());
            }
        } else if (key == "timeStampNs" || key == "timestampNs") {
            auto timestamp = field_result.value().get_int64();
            if (!timestamp.error()) output.timestamp_ns = timestamp.value_unsafe();
        } else if ((key == "pose" || key == "p") && !pose_ok) {
            const char* text = nullptr;
            std::size_t length = 0;
            if (get_text(field_result.value(), text, length)) {
                pose_ok = get_pose_text(text, length, output.pose);
            }
        } else if (key == "va" || key == "wva") {
            const char* text = nullptr;
            std::size_t length = 0;
            if (get_text(field_result.value(), text, length)) {
                if (key == "va") get_vec6_text(text, length, output.linear_velocity, output.angular_velocity);
                else get_vec6_text(text, length, output.linear_acceleration, output.angular_acceleration);
            }
        } else if (radius != nullptr && key == "r") {
            auto number = field_result.value().get_double();
            if (!number.error()) *radius = static_cast<float>(number.value_unsafe());
        } else if (extra_timestamp != nullptr && key == "t") {
            auto ts = field_result.value().get_int64();
            if (!ts.error()) *extra_timestamp = ts.value_unsafe();
        } else if (controller != nullptr) {
            if (key == "axisX" || key == "axisY" || key == "grip" || key == "trigger") {
                auto number = field_result.value().get_double();
                if (number.error()) continue;
                const float v = static_cast<float>(number.value_unsafe());
                if (key == "axisX") controller->axis_x = v;
                else if (key == "axisY") controller->axis_y = v;
                else if (key == "grip") controller->grip = v;
                else controller->trigger = v;
            } else if (key == "axisClick" || key == "primaryButton" || key == "secondaryButton" || key == "menuButton") {
                bool button = false;
                auto boolean = field_result.value().get_bool();
                if (!boolean.error()) button = boolean.value_unsafe();
                else {
                    auto integer = field_result.value().get_uint64();
                    if (!integer.error()) button = integer.value_unsafe() != 0;
                }
                if (key == "axisClick") controller->axis_click = button;
                else if (key == "primaryButton") controller->primary_button = button;
                else if (key == "secondaryButton") controller->secondary_button = button;
                else controller->menu_button = button;
            }
        }
    }
    output.status = status;
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
    parse_state_object(object, timestamp, output.tracking, &output);
}

void parse_hand(simdjson::ondemand::value value, pico_dds_HandState& output) {
    auto object_result = value.get_object();
    if (object_result.error()) return;
    simdjson::ondemand::object object = object_result.value_unsafe();
    output.scale = 1.0f;
    for (auto field_result : object) {
        auto key_result = field_result.unescaped_key();
        if (key_result.error()) continue;
        const auto key = key_result.value_unsafe();
        if (key == "isActive") {
            auto integer = field_result.value().get_uint64();
            if (!integer.error()) output.active = integer.value_unsafe() != 0;
        } else if (key == "scale") {
            auto number = field_result.value().get_double();
            if (!number.error()) output.scale = static_cast<float>(number.value_unsafe());
        } else if (key == "timeStampNs" || key == "timestampNs") {
            auto timestamp = field_result.value().get_int64();
            if (!timestamp.error()) output.timestamp_ns = timestamp.value_unsafe();
        } else if (key == "HandJointLocations" || key == "joints") {
            auto joints_result = field_result.value().get_array();
            if (joints_result.error()) continue;
            std::size_t index = 0;
            for (auto joint_result : joints_result.value_unsafe()) {
                if (index >= 26) break;
                if (joint_result.error()) continue;
                simdjson::ondemand::value joint = joint_result.value_unsafe();
                auto joint_object = joint.get_object();
                if (joint_object.error()) continue;
                parse_state_object(joint_object.value_unsafe(), output.timestamp_ns,
                                   output.joints[index].tracking, nullptr,
                                   &output.joints[index].radius);
                ++index;
            }
            output.count = static_cast<std::uint32_t>(index);
        }
    }
}

void parse_body(simdjson::ondemand::value value, std::int64_t timestamp, pico_dds_TrackingFrame& output) {
    auto object_result = value.get_object();
    if (object_result.error()) return;
    simdjson::ondemand::object object = object_result.value_unsafe();
    std::int64_t body_timestamp = timestamp;
    std::size_t index = 0;
    for (auto field_result : object) {
        auto key_result = field_result.unescaped_key();
        if (key_result.error()) continue;
        const auto key = key_result.value_unsafe();
        if (key == "timeStampNs" || key == "timestampNs") {
            auto ts = field_result.value().get_int64();
            if (!ts.error()) body_timestamp = ts.value_unsafe();
        } else if (key == "joints") {
            auto joints = field_result.value().get_array();
            if (joints.error()) continue;
            for (auto joint_result : joints.value_unsafe()) {
                if (index >= 24) break;
                if (joint_result.error()) continue;
                simdjson::ondemand::value joint = joint_result.value_unsafe();
                auto joint_object = joint.get_object();
                if (joint_object.error()) continue;
                output.body[index].role = static_cast<std::uint32_t>(index);
                parse_state_object(joint_object.value_unsafe(), body_timestamp,
                                   output.body[index].tracking, nullptr, nullptr,
                                   &output.body[index].imu_timestamp_ns);
                ++index;
            }
        }
    }
    output.body_count = static_cast<std::uint32_t>(index);
}

void parse_motion(simdjson::ondemand::value value, std::int64_t timestamp, pico_dds_TrackingFrame& output) {
    auto object_result = value.get_object();
    if (object_result.error()) return;
    simdjson::ondemand::object object = object_result.value_unsafe();
    std::size_t index = 0;
    for (auto field_result : object) {
        auto key_result = field_result.unescaped_key();
        if (key_result.error()) continue;
        const auto key = key_result.value_unsafe();
        if (key != "joints") continue;
        auto joints = field_result.value().get_array();
        if (joints.error()) continue;
        for (auto joint_result : joints.value_unsafe()) {
            if (index >= 5) break;
            if (joint_result.error()) continue;
            simdjson::ondemand::value joint = joint_result.value_unsafe();
            auto joint_object = joint.get_object();
            if (joint_object.error()) continue;
            output.trackers[index].index = static_cast<std::uint32_t>(index);
            parse_state_object(joint_object.value_unsafe(), timestamp,
                               output.trackers[index].tracking);
            ++index;
        }
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
                // Single ordered pass: looking up both children before parsing
                // would consume the first child's value (ondemand is
                // forward-only), so dispatch each child as it is reached.
                for (auto child : object_result.value_unsafe()) {
                    auto child_key = child.unescaped_key();
                    if (child_key.error()) continue;
                    const auto name = child_key.value_unsafe();
                    if (name == "left" || name == "Left") {
                        parse_controller(child.value(), output.source_timestamp_ns, output.left_controller);
                    } else if (name == "right" || name == "Right") {
                        parse_controller(child.value(), output.source_timestamp_ns, output.right_controller);
                    }
                }
            }
        } else if (key == "Hand" || key == "hand") {
            auto object_result = root_field.value().get_object();
            if (!object_result.error()) {
                for (auto child : object_result.value_unsafe()) {
                    auto child_key = child.unescaped_key();
                    if (child_key.error()) continue;
                    const auto name = child_key.value_unsafe();
                    if (name == "leftHand" || name == "left") {
                        parse_hand(child.value(), output.left_hand);
                    } else if (name == "rightHand" || name == "right") {
                        parse_hand(child.value(), output.right_hand);
                    }
                }
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
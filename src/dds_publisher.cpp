#include "pico_dds_bridge/dds_publisher.hpp"

#include "pico_tracking.h"

#include <stdexcept>
#include <string>
#include <utility>

namespace pico_dds_bridge {
namespace {

void check_entity(const dds_entity_t entity, const char* what) {
    if (entity < 0) {
        throw std::runtime_error(
            std::string(what) + " failed, DDS rc=" + std::to_string(entity));
    }
}

pico_dds_Vec3 to_dds(const Vec3& value) {
    pico_dds_Vec3 out{};
    out.x = value.x;
    out.y = value.y;
    out.z = value.z;
    return out;
}

pico_dds_Quaternion to_dds(const Quaternion& value) {
    pico_dds_Quaternion out{};
    out.x = value.x;
    out.y = value.y;
    out.z = value.z;
    out.w = value.w;
    return out;
}

pico_dds_Pose to_dds(const Pose& value) {
    pico_dds_Pose out{};
    out.position = to_dds(value.position);
    out.orientation = to_dds(value.orientation);
    return out;
}

pico_dds_TrackingState to_dds(const TrackingState& value) {
    pico_dds_TrackingState out{};
    out.valid = value.valid;
    out.status = value.status;
    out.timestamp_ns = value.timestamp_ns;
    out.pose = to_dds(value.pose);
    out.linear_velocity = to_dds(value.linear_velocity);
    out.angular_velocity = to_dds(value.angular_velocity);
    out.linear_acceleration = to_dds(value.linear_acceleration);
    out.angular_acceleration = to_dds(value.angular_acceleration);
    return out;
}

pico_dds_ControllerState to_dds(const ControllerState& value) {
    pico_dds_ControllerState out{};
    out.tracking = to_dds(value.tracking);
    out.axis_x = value.axis_x;
    out.axis_y = value.axis_y;
    out.grip = value.grip;
    out.trigger = value.trigger;
    out.axis_click = value.axis_click;
    out.primary_button = value.primary_button;
    out.secondary_button = value.secondary_button;
    out.menu_button = value.menu_button;
    return out;
}

pico_dds_HandJoint to_dds(const HandJoint& value) {
    pico_dds_HandJoint out{};
    out.tracking = to_dds(value.tracking);
    out.radius = value.radius;
    return out;
}

pico_dds_HandState to_dds(const HandState& value) {
    pico_dds_HandState out{};
    out.active = value.active;
    out.count = value.count;
    out.scale = value.scale;
    out.timestamp_ns = value.timestamp_ns;

    for (std::size_t i = 0; i < value.joints.size(); ++i) {
        out.joints[i] = to_dds(value.joints[i]);
    }

    return out;
}

pico_dds_BodyJoint to_dds(const BodyJoint& value) {
    pico_dds_BodyJoint out{};
    out.role = value.role;
    out.tracking = to_dds(value.tracking);
    out.imu_timestamp_ns = value.imu_timestamp_ns;
    return out;
}

pico_dds_MotionTracker to_dds(const MotionTracker& value) {
    pico_dds_MotionTracker out{};
    out.index = value.index;
    out.tracking = to_dds(value.tracking);
    return out;
}

pico_dds_TrackingFrame to_dds(const TrackingFrame& value) {
    pico_dds_TrackingFrame out{};

    out.frame_seq = value.sequence;
    out.source_timestamp_ns = value.source_timestamp_ns;
    out.receive_timestamp_ns = value.receive_timestamp_ns;
    out.input_mode = value.input_mode;

    out.head = to_dds(value.head);
    out.left_controller = to_dds(value.left_controller);
    out.right_controller = to_dds(value.right_controller);

    out.left_hand = to_dds(value.left_hand);
    out.right_hand = to_dds(value.right_hand);

    out.body_count = value.body_count;
    for (std::size_t i = 0; i < value.body.size(); ++i) {
        out.body[i] = to_dds(value.body[i]);
    }

    out.tracker_count = value.tracker_count;
    for (std::size_t i = 0; i < value.trackers.size(); ++i) {
        out.trackers[i] = to_dds(value.trackers[i]);
    }

    return out;
}

}  // namespace

DdsPublisher::DdsPublisher(
    const std::uint32_t domain_id,
    std::string topic_name) {

    participant_ = dds_create_participant(domain_id, nullptr, nullptr);
    check_entity(participant_, "dds_create_participant");

    topic_ = dds_create_topic(
        participant_,
        &pico_dds_TrackingFrame_desc,
        topic_name.c_str(),
        nullptr,
        nullptr);
    check_entity(topic_, "dds_create_topic");

    dds_qos_t* qos = dds_create_qos();
    if (qos == nullptr) {
        throw std::runtime_error("dds_create_qos failed");
    }

    // Tracking data: reliable, shallow history. If the pipeline stalls we want
    // current state, not seconds of stale teleoperation frames.
    dds_qset_reliability(
        qos,
        DDS_RELIABILITY_RELIABLE,
        DDS_MSECS(50));
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 4);

    writer_ = dds_create_writer(participant_, topic_, qos, nullptr);
    dds_delete_qos(qos);
    check_entity(writer_, "dds_create_writer");
}

DdsPublisher::~DdsPublisher() {
    if (participant_ != DDS_ENTITY_NIL) {
        (void)dds_delete(participant_);
    }
}

void DdsPublisher::publish(const TrackingFrame& frame) {
    auto sample = to_dds(frame);
    const dds_return_t rc = dds_write(writer_, &sample);

    if (rc < 0) {
        throw std::runtime_error(
            "dds_write failed, DDS rc=" + std::to_string(rc));
    }
}

}  // namespace pico_dds_bridge

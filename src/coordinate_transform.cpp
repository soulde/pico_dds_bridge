#include "pico_dds_bridge/coordinate_transform.hpp"

namespace pico_dds_bridge {
namespace {

Vec3 convert_vector(const Vec3& value) {
    return Vec3{value.z, -value.x, value.y};
}

Quaternion convert_orientation(const Quaternion& value) {
    return Quaternion{value.z, -value.x, value.y, value.w};
}

void convert_tracking_state(TrackingState& state) {
    state.pose.position = convert_vector(state.pose.position);
    state.pose.orientation = convert_orientation(state.pose.orientation);
    state.linear_velocity = convert_vector(state.linear_velocity);
    state.angular_velocity = convert_vector(state.angular_velocity);
    state.linear_acceleration = convert_vector(state.linear_acceleration);
    state.angular_acceleration = convert_vector(state.angular_acceleration);
}

void convert_hand(HandState& hand) {
    for (auto& joint : hand.joints) {
        convert_tracking_state(joint.tracking);
    }
}

}  // namespace

void convert_to_robot_coordinates(TrackingFrame& frame) {
    convert_tracking_state(frame.head);
    convert_tracking_state(frame.left_controller.tracking);
    convert_tracking_state(frame.right_controller.tracking);
    convert_hand(frame.left_hand);
    convert_hand(frame.right_hand);

    for (auto& joint : frame.body) {
        convert_tracking_state(joint.tracking);
    }

    for (auto& tracker : frame.trackers) {
        convert_tracking_state(tracker.tracking);
    }
}

}  // namespace pico_dds_bridge
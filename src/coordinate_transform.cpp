#include "pico_dds_bridge/coordinate_transform.hpp"

#include <cstddef>

namespace pico_dds_bridge {
namespace {

void convert_vector(pico_dds_Vec3& value) {
    const double x = value.x;
    const double y = value.y;
    const double z = value.z;
    value.x = z;
    value.y = -x;
    value.z = y;
}

void convert_orientation(pico_dds_Quaternion& value) {
    const double x = value.x;
    const double y = value.y;
    const double z = value.z;
    value.x = z;
    value.y = -x;
    value.z = y;
}

void convert_tracking_state(pico_dds_TrackingState& state) {
    convert_vector(state.pose.position);
    convert_orientation(state.pose.orientation);
    convert_vector(state.linear_velocity);
    convert_vector(state.angular_velocity);
    convert_vector(state.linear_acceleration);
    convert_vector(state.angular_acceleration);
}

void convert_hand(pico_dds_HandState& hand) {
    for (std::size_t i = 0; i < 26; ++i) {
        pico_dds_HandJoint& joint = hand.joints[i];
        convert_tracking_state(joint.tracking);
    }
}

}  // namespace

void convert_to_robot_coordinates(pico_dds_TrackingFrame& frame) {
    convert_tracking_state(frame.head);
    convert_tracking_state(frame.left_controller.tracking);
    convert_tracking_state(frame.right_controller.tracking);
    convert_hand(frame.left_hand);
    convert_hand(frame.right_hand);

    for (std::size_t i = 0; i < 24; ++i) {
        pico_dds_BodyJoint& joint = frame.body[i];
        convert_tracking_state(joint.tracking);
    }

    for (std::size_t i = 0; i < 5; ++i) {
        pico_dds_MotionTracker& tracker = frame.trackers[i];
        convert_tracking_state(tracker.tracking);
    }
}

}  // namespace pico_dds_bridge
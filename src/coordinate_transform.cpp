#include "pico_dds_bridge/coordinate_transform.hpp"

#include <cstddef>

namespace pico_dds_bridge {
namespace {

// Proper right-handed rotation (det=+1) from PICO tracking axes to robot
// axes (X forward, Y left, Z up). A pure axis permutation like (z, -x, y)
// is a reflection (det=-1) and mirrors left/right.
void convert_robot_vector(pico_dds_Vec3& value) {
    const double x = value.x;
    const double y = value.y;
    const double z = value.z;
    value.x = -z;
    value.y = -x;
    value.z = y;
}

void convert_robot_orientation(pico_dds_Quaternion& value) {
    const double x = value.x;
    const double y = value.y;
    const double z = value.z;
    value.x = -z;
    value.y = -x;
    value.z = y;
}

void convert_xrobot_vector(pico_dds_Vec3& value) {
    const double x = value.x;
    const double y = value.y;
    const double z = value.z;
    value.x = x;
    value.y = -z;
    value.z = y;
}

void convert_xrobot_orientation(pico_dds_Quaternion& value) {
    // Match XRobotStreamer.coordinate_transform_unity_data exactly:
    // q_out = q_rotation_matrix * q_input, scalar-first.
    constexpr double s = 0.7071067811865476;
    const double w = value.w;
    const double x = value.x;
    const double y = value.y;
    const double z = value.z;
    value.w = s * (w - x);
    value.x = s * (w + x);
    value.y = s * (y - z);
    value.z = s * (y + z);
}

void convert_tracking_state(pico_dds_TrackingState& state, CoordinateSystem system) {
    if (system == CoordinateSystem::Pico) return;

    const bool xrobot = system == CoordinateSystem::Xrobot;
    auto convert_vector = xrobot ? convert_xrobot_vector : convert_robot_vector;
    auto convert_orientation = xrobot ? convert_xrobot_orientation : convert_robot_orientation;
    convert_vector(state.pose.position);
    convert_orientation(state.pose.orientation);
    convert_vector(state.linear_velocity);
    convert_vector(state.angular_velocity);
    convert_vector(state.linear_acceleration);
    convert_vector(state.angular_acceleration);
}

void convert_hand(pico_dds_HandState& hand, CoordinateSystem system) {
    for (std::size_t i = 0; i < 26; ++i) {
        pico_dds_HandJoint& joint = hand.joints[i];
        convert_tracking_state(joint.tracking, system);
    }
}

}  // namespace

void convert_to_coordinates(pico_dds_TrackingFrame& frame, CoordinateSystem system) {
    convert_tracking_state(frame.head, system);
    convert_tracking_state(frame.left_controller.tracking, system);
    convert_tracking_state(frame.right_controller.tracking, system);
    convert_hand(frame.left_hand, system);
    convert_hand(frame.right_hand, system);

    for (std::size_t i = 0; i < 24; ++i) {
        pico_dds_BodyJoint& joint = frame.body[i];
        convert_tracking_state(joint.tracking, system);
    }

    for (std::size_t i = 0; i < 5; ++i) {
        pico_dds_MotionTracker& tracker = frame.trackers[i];
        convert_tracking_state(tracker.tracking, system);
    }
}

}  // namespace pico_dds_bridge

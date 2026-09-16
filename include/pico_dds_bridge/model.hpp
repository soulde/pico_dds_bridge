#pragma once

#include <array>
#include <cstdint>

namespace pico_dds_bridge {

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

struct Quaternion {
    double x{};
    double y{};
    double z{};
    double w{1.0};
};

struct Pose {
    Vec3 position{};
    Quaternion orientation{};
};

struct TrackingState {
    bool valid{};
    std::uint32_t status{};
    std::int64_t timestamp_ns{};

    Pose pose{};

    Vec3 linear_velocity{};
    Vec3 angular_velocity{};
    Vec3 linear_acceleration{};
    Vec3 angular_acceleration{};
};

struct ControllerState {
    TrackingState tracking{};

    float axis_x{};
    float axis_y{};
    float grip{};
    float trigger{};

    bool axis_click{};
    bool primary_button{};
    bool secondary_button{};
    bool menu_button{};
};

struct HandJoint {
    TrackingState tracking{};
    float radius{};
};

struct HandState {
    bool active{};
    std::uint32_t count{};
    float scale{1.0F};
    std::int64_t timestamp_ns{};
    std::array<HandJoint, 26> joints{};
};

struct BodyJoint {
    std::uint32_t role{};
    TrackingState tracking{};
    std::int64_t imu_timestamp_ns{};
};

struct MotionTracker {
    std::uint32_t index{};
    TrackingState tracking{};
};

struct TrackingFrame {
    std::uint64_t sequence{};
    std::int64_t source_timestamp_ns{};
    std::int64_t receive_timestamp_ns{};
    std::int32_t input_mode{-1};

    TrackingState head{};
    ControllerState left_controller{};
    ControllerState right_controller{};

    HandState left_hand{};
    HandState right_hand{};

    std::uint32_t body_count{};
    std::array<BodyJoint, 24> body{};

    std::uint32_t tracker_count{};
    std::array<MotionTracker, 5> trackers{};
};

}  // namespace pico_dds_bridge

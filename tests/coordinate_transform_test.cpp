#include "pico_dds_bridge/coordinate_transform.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>

namespace {

bool close(const double a, const double b) {
    return std::abs(a - b) < 1e-9;
}

void expect_vec(const pico_dds_Vec3& value, double x, double y, double z) {
    if (!close(value.x, x) || !close(value.y, y) || !close(value.z, z)) {
        std::abort();
    }
}

}  // namespace

int main() {
    using pico_dds_bridge::CoordinateSystem;
    using TrackingFrame = pico_dds_TrackingFrame;

    TrackingFrame pico{};
    pico.body[0].tracking.pose.position = {1.0, 2.0, 3.0};
    pico.body[0].tracking.pose.orientation = {0.1, 0.2, 0.3, 0.9};
    pico_dds_bridge::convert_to_coordinates(pico, CoordinateSystem::Pico);
    expect_vec(pico.body[0].tracking.pose.position, 1.0, 2.0, 3.0);

    TrackingFrame xrobot{};
    xrobot.body[0].tracking.pose.position = {1.0, 2.0, 3.0};
    xrobot.body[0].tracking.pose.orientation = {0.1, 0.2, 0.3, 0.9};
    pico_dds_bridge::convert_to_coordinates(xrobot, CoordinateSystem::Xrobot);
    expect_vec(xrobot.body[0].tracking.pose.position, 1.0, -3.0, 2.0);
    expect_vec(
        pico_dds_Vec3{xrobot.body[0].tracking.pose.orientation.x,
                      xrobot.body[0].tracking.pose.orientation.y,
                      xrobot.body[0].tracking.pose.orientation.z},
        0.7071067811865476, -0.07071067811865475, 0.3535533905932738);
    if (!close(xrobot.body[0].tracking.pose.orientation.w, 0.565685424949238)) {
        std::abort();
    }

    TrackingFrame robot{};
    robot.body[0].tracking.pose.position = {1.0, 2.0, 3.0};
    pico_dds_bridge::convert_to_coordinates(robot, CoordinateSystem::Robot);
    expect_vec(robot.body[0].tracking.pose.position, -3.0, -1.0, 2.0);
}

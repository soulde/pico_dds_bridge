#pragma once

#include "pico_dds_bridge/model.hpp"

namespace pico_dds_bridge {

void convert_to_robot_coordinates(TrackingFrame& frame);

}  // namespace pico_dds_bridge
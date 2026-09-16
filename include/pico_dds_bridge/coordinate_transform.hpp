#pragma once

#include "pico_tracking.h"

namespace pico_dds_bridge {

void convert_to_robot_coordinates(pico_dds_TrackingFrame& frame);

}  // namespace pico_dds_bridge
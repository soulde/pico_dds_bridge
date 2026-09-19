#pragma once

#include "pico_tracking.h"

namespace pico_dds_bridge {

enum class CoordinateSystem {
    Pico,
    Robot,
    Xrobot,
};

void convert_to_coordinates(pico_dds_TrackingFrame& frame, CoordinateSystem system);

}  // namespace pico_dds_bridge

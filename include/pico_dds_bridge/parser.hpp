#pragma once

#include "pico_dds_bridge/model.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace pico_dds_bridge {

class TrackingParser {
public:
    std::optional<TrackingFrame> parse(
        std::string_view callback_json,
        std::uint64_t sequence,
        std::int64_t receive_timestamp_ns) const;
};

}  // namespace pico_dds_bridge

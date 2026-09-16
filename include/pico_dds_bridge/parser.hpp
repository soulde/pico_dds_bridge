#pragma once

#include "pico_dds_bridge/model.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace pico_dds_bridge {

class TrackingParser {
public:
    std::unique_ptr<TrackingFrame> parse(
        const std::string& callback_json,
        std::uint64_t sequence,
        std::int64_t receive_timestamp_ns) const;
};

}  // namespace pico_dds_bridge

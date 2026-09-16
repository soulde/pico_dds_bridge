#pragma once

#include "pico_tracking.h"

#include <cstdint>
#include <cstddef>

namespace pico_dds_bridge {

class TrackingParser {
public:
    TrackingParser();
    ~TrackingParser();

    TrackingParser(const TrackingParser&) = delete;
    TrackingParser& operator=(const TrackingParser&) = delete;

    bool parse_into(
        const char* json,
        std::size_t length,
        std::size_t capacity,
        std::uint64_t sequence,
        std::int64_t receive_timestamp_ns,
        pico_dds_TrackingFrame& out);

private:
    struct Impl;
    Impl* impl_;
};

}  // namespace pico_dds_bridge

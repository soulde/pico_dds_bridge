#pragma once

#include "pico_dds_bridge/model.hpp"

#include <dds/dds.h>

#include <cstdint>
#include <string>

namespace pico_dds_bridge {

class DdsPublisher {
public:
    explicit DdsPublisher(
        std::uint32_t domain_id = DDS_DOMAIN_DEFAULT,
        std::string topic_name = "pico/tracking");

    ~DdsPublisher();

    DdsPublisher(const DdsPublisher&) = delete;
    DdsPublisher& operator=(const DdsPublisher&) = delete;

    void publish(const TrackingFrame& frame);

private:
    dds_entity_t participant_{DDS_ENTITY_NIL};
    dds_entity_t topic_{DDS_ENTITY_NIL};
    dds_entity_t writer_{DDS_ENTITY_NIL};
};

}  // namespace pico_dds_bridge

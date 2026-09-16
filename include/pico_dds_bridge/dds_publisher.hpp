#pragma once

#include <dds/dds.h>
#include "pico_tracking.h"

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

    pico_dds_TrackingFrame* request_sample();
    void publish(pico_dds_TrackingFrame* sample);
    void cancel(pico_dds_TrackingFrame*& sample);

    bool shared_memory_available() const noexcept {
        return shared_memory_available_;
    }

private:
    dds_entity_t participant_{DDS_ENTITY_NIL};
    dds_entity_t topic_{DDS_ENTITY_NIL};
    dds_entity_t writer_{DDS_ENTITY_NIL};
    bool shared_memory_available_{false};
};

}  // namespace pico_dds_bridge

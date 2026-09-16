#include "pico_dds_bridge/dds_publisher.hpp"

#include "pico_tracking.h"

#include <cstring>
#include <stdexcept>
#include <string>

namespace pico_dds_bridge {
namespace {

void check_entity(const dds_entity_t entity, const char* what) {
    if (entity < 0) {
        throw std::runtime_error(
            std::string(what) + " failed, DDS rc=" + std::to_string(entity));
    }
}

}  // namespace

DdsPublisher::DdsPublisher(
    const std::uint32_t domain_id,
    std::string topic_name) {

    participant_ = dds_create_participant(domain_id, nullptr, nullptr);
    check_entity(participant_, "dds_create_participant");

    topic_ = dds_create_topic(
        participant_,
        &pico_dds_TrackingFrame_desc,
        topic_name.c_str(),
        nullptr,
        nullptr);
    check_entity(topic_, "dds_create_topic");

    dds_qos_t* qos = dds_create_qos();
    if (qos == nullptr) {
        throw std::runtime_error("dds_create_qos failed");
    }

    // Tracking is a latest-state stream: stale samples have no value.
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
    dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);

    writer_ = dds_create_writer(participant_, topic_, qos, nullptr);
    dds_delete_qos(qos);
    check_entity(writer_, "dds_create_writer");
    shared_memory_available_ = dds_is_shared_memory_available(writer_);
}

DdsPublisher::~DdsPublisher() {
    if (participant_ != DDS_ENTITY_NIL) {
        (void)dds_delete(participant_);
    }
}

pico_dds_TrackingFrame* DdsPublisher::request_sample() {
    void* ptr = nullptr;
    const dds_return_t rc = dds_request_loan(writer_, &ptr);
    if (rc != DDS_RETCODE_OK || ptr == nullptr) {
        return nullptr;
    }

    auto* sample = static_cast<pico_dds_TrackingFrame*>(ptr);
    std::memset(sample, 0, sizeof(*sample));
    return sample;
}

void DdsPublisher::publish(pico_dds_TrackingFrame* sample) {
    if (sample == nullptr) {
        throw std::runtime_error("dds_write called with null sample");
    }
    const dds_return_t rc = dds_write(writer_, sample);

    if (rc < 0) {
        throw std::runtime_error(
            "dds_write failed, DDS rc=" + std::to_string(rc));
    }
}

void DdsPublisher::cancel(pico_dds_TrackingFrame*& sample) {
    if (sample == nullptr) {
        return;
    }
    void* ptr = sample;
    const dds_return_t rc = dds_return_loan(writer_, &ptr, 1);
    sample = nullptr;
    if (rc < 0) {
        throw std::runtime_error(
            "dds_return_loan failed, DDS rc=" + std::to_string(rc));
    }
}

}  // namespace pico_dds_bridge

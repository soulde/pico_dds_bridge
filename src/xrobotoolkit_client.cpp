#include "pico_dds_bridge/xrobotoolkit_client.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <utility>

namespace pico_dds_bridge {

XRoboToolkitClient::XRoboToolkitClient(const std::size_t queue_capacity)
    : queue_capacity_(std::min(queue_capacity, kQueueDepth)) {
    for (std::size_t i = 0; i < kQueueDepth; ++i) {
        slots_[i].data.reset(new char[kMaxJsonSize + simdjson::SIMDJSON_PADDING]);
        states_[i] = 0;
    }
}

XRoboToolkitClient::~XRoboToolkitClient() {
    stop();
}

bool XRoboToolkitClient::start() {
    bool expected = false;
    if (!started_.compare_exchange_strong(expected, true)) {
        return true;
    }

    const int rc = PXREAInit(this, &XRoboToolkitClient::callback_thunk, PXREAFullMask);
    if (rc != 0) {
        started_.store(false);
        std::cerr << "[xrobotoolkit] PXREAInit failed, rc=" << rc << '\n';
        return false;
    }

    return true;
}

void XRoboToolkitClient::stop() {
    if (!started_.exchange(false)) {
        return;
    }

    (void)PXREADeinit();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_head_ = 0;
        ready_tail_ = 0;
        ready_size_ = 0;
        for (std::size_t i = 0; i < kQueueDepth; ++i) {
            states_[i] = 0;
        }
    }
    cv_.notify_all();
    server_connected_.store(false);
}

RawFrameSlot* XRoboToolkitClient::wait_pop(
    const std::chrono::milliseconds timeout) {

    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, timeout, [this] {
        return ready_size_ != 0 || !started_.load();
    });

    if (ready_size_ == 0) {
        return nullptr;
    }

    const std::size_t index = ready_[ready_head_];
    ready_head_ = (ready_head_ + 1) % kQueueDepth;
    --ready_size_;
    states_[index] = 2;
    return &slots_[index];
}

void XRoboToolkitClient::release(RawFrameSlot* slot) {
    if (slot == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t index = static_cast<std::size_t>(slot - slots_.data());
    if (index < kQueueDepth) {
        states_[index] = 0;
    }
}

void XRoboToolkitClient::callback_thunk(
    void* context,
    const PXREAClientCallbackType type,
    const int status,
    void* user_data) {

    if (context == nullptr) {
        return;
    }

    static_cast<XRoboToolkitClient*>(context)->on_callback(type, status, user_data);
}

void XRoboToolkitClient::on_callback(
    const PXREAClientCallbackType type,
    const int /*status*/,
    void* user_data) {

    switch (type) {
    case PXREAServerConnect:
        server_connected_.store(true);
        std::cerr << "[xrobotoolkit] PC Service connected\n";
        return;

    case PXREAServerDisconnect:
        server_connected_.store(false);
        std::cerr << "[xrobotoolkit] PC Service disconnected\n";
        return;

    case PXREADeviceStateJson:
        break;

    case PXREADeviceFind:
        if (user_data != nullptr) {
            std::cerr << "[xrobotoolkit] device online: "
                      << static_cast<const char*>(user_data) << '\n';
        }
        return;

    case PXREADeviceMissing:
        if (user_data != nullptr) {
            std::cerr << "[xrobotoolkit] device offline: "
                      << static_cast<const char*>(user_data) << '\n';
        }
        return;

    default:
        return;
    }

    if (user_data == nullptr || !started_.load()) {
        return;
    }

    // Important: SDK owns user_data. Copy it inside the callback immediately.
    {
        std::lock_guard<std::mutex> lock(mutex_);

        std::size_t index = kQueueDepth;
        for (std::size_t i = 0; i < queue_capacity_; ++i) {
            if (states_[i] == 0) {
                index = i;
                break;
            }
        }

        if (index == kQueueDepth && ready_size_ >= queue_capacity_) {
            const std::size_t old_index = ready_[ready_head_];
            ready_head_ = (ready_head_ + 1) % kQueueDepth;
            --ready_size_;
            states_[old_index] = 0;
            index = old_index;
            dropped_frames_.fetch_add(1, std::memory_order_relaxed);
        }

        if (index == kQueueDepth) {
            dropped_frames_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        RawFrameSlot& frame = slots_[index];
        const char* source = static_cast<const char*>(user_data);
        const std::size_t length = std::strlen(source);
        if (length > kMaxJsonSize) {
            dropped_frames_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        std::memcpy(frame.data.get(), source, length);
        std::memset(
            frame.data.get() + length,
            0,
            simdjson::SIMDJSON_PADDING);
        frame.length = length;
        frame.receive_timestamp_ns = realtime_now_ns();
        states_[index] = 1;

        if (ready_size_ >= queue_capacity_) {
            const std::size_t old_index = ready_[ready_head_];
            ready_head_ = (ready_head_ + 1) % kQueueDepth;
            --ready_size_;
            states_[old_index] = 0;
            dropped_frames_.fetch_add(1, std::memory_order_relaxed);
        }

        ready_[ready_tail_] = index;
        ready_tail_ = (ready_tail_ + 1) % kQueueDepth;
        ++ready_size_;
    }

    cv_.notify_one();
}

std::int64_t XRoboToolkitClient::realtime_now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(
        system_clock::now().time_since_epoch()).count();
}

}  // namespace pico_dds_bridge

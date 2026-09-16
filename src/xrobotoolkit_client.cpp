#include "pico_dds_bridge/xrobotoolkit_client.hpp"

#include <chrono>
#include <iostream>
#include <utility>

namespace pico_dds_bridge {

XRoboToolkitClient::XRoboToolkitClient(const std::size_t queue_capacity)
    : queue_capacity_(queue_capacity) {}

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
        std::lock_guard lock(mutex_);
        queue_.clear();
    }
    cv_.notify_all();
    server_connected_.store(false);
}

std::optional<RawFrame> XRoboToolkitClient::wait_pop(
    const std::chrono::milliseconds timeout) {

    std::unique_lock lock(mutex_);
    cv_.wait_for(lock, timeout, [this] {
        return !queue_.empty() || !started_.load();
    });

    if (queue_.empty()) {
        return std::nullopt;
    }

    RawFrame frame = std::move(queue_.front());
    queue_.pop_front();
    return frame;
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
    RawFrame frame{
        .json = std::string(static_cast<const char*>(user_data)),
        .receive_timestamp_ns = realtime_now_ns(),
    };

    {
        std::lock_guard lock(mutex_);

        // Tracking is real-time data. If consumers fall behind, keep the newest
        // samples instead of accumulating latency.
        if (queue_.size() >= queue_capacity_) {
            queue_.pop_front();
            dropped_frames_.fetch_add(1, std::memory_order_relaxed);
        }

        queue_.push_back(std::move(frame));
    }

    cv_.notify_one();
}

std::int64_t XRoboToolkitClient::realtime_now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(
        system_clock::now().time_since_epoch()).count();
}

}  // namespace pico_dds_bridge

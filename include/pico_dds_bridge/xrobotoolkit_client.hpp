#pragma once
#include <cstdint> 
#include <PXREARobotSDK.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

namespace pico_dds_bridge {

struct RawFrame {
    std::string json;
    std::int64_t receive_timestamp_ns{};
};

class XRoboToolkitClient {
public:
    explicit XRoboToolkitClient(std::size_t queue_capacity = 8);
    ~XRoboToolkitClient();

    XRoboToolkitClient(const XRoboToolkitClient&) = delete;
    XRoboToolkitClient& operator=(const XRoboToolkitClient&) = delete;

    bool start();
    void stop();

    std::unique_ptr<RawFrame> wait_pop(std::chrono::milliseconds timeout);

    bool server_connected() const noexcept {
        return server_connected_.load(std::memory_order_relaxed);
    }

    std::uint64_t dropped_frames() const noexcept {
        return dropped_frames_.load(std::memory_order_relaxed);
    }

private:
    static void callback_thunk(
        void* context,
        PXREAClientCallbackType type,
        int status,
        void* user_data);

    void on_callback(
        PXREAClientCallbackType type,
        int status,
        void* user_data);

    static std::int64_t realtime_now_ns();

    const std::size_t queue_capacity_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<RawFrame> queue_;

    std::atomic_bool started_{false};
    std::atomic_bool server_connected_{false};
    std::atomic<std::uint64_t> dropped_frames_{0};
};

}  // namespace pico_dds_bridge

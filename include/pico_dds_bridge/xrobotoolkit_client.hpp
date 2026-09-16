#pragma once
#include <cstdint> 
#include <PXREARobotSDK.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <array>

#include <simdjson.h>

namespace pico_dds_bridge {

static constexpr std::size_t kQueueDepth = 8;
static constexpr std::size_t kMaxJsonSize = 256U * 1024U;

struct RawFrameSlot {
    std::unique_ptr<char[]> data;
    std::size_t length{0};
    std::int64_t receive_timestamp_ns{0};
    std::size_t capacity{kMaxJsonSize + simdjson::SIMDJSON_PADDING};
};

class XRoboToolkitClient {
public:
    explicit XRoboToolkitClient(std::size_t queue_capacity = kQueueDepth);
    ~XRoboToolkitClient();

    XRoboToolkitClient(const XRoboToolkitClient&) = delete;
    XRoboToolkitClient& operator=(const XRoboToolkitClient&) = delete;

    bool start();
    void stop();

    RawFrameSlot* wait_pop(std::chrono::milliseconds timeout);
    void release(RawFrameSlot* slot);

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
    std::array<RawFrameSlot, kQueueDepth> slots_;
    std::array<std::size_t, kQueueDepth> ready_{};
    std::array<unsigned char, kQueueDepth> states_{};
    std::size_t ready_head_{0};
    std::size_t ready_tail_{0};
    std::size_t ready_size_{0};

    std::atomic_bool started_{false};
    std::atomic_bool server_connected_{false};
    std::atomic<std::uint64_t> dropped_frames_{0};
};

}  // namespace pico_dds_bridge

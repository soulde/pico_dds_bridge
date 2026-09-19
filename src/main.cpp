#include "pico_dds_bridge/dds_publisher.hpp"
#include "pico_dds_bridge/coordinate_transform.hpp"
#include "pico_dds_bridge/parser.hpp"
#include "pico_dds_bridge/xrobotoolkit_client.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <chrono>

namespace {

std::atomic_bool g_running{true};

void on_signal(int) {
    g_running.store(false);
}

struct Options {
    std::uint32_t domain_id{0};
    std::string topic{"pico/tracking"};
    pico_dds_bridge::CoordinateSystem coordinates{
        pico_dds_bridge::CoordinateSystem::Pico};
    bool require_shm{false};
};

Options parse_options(const int argc, char** argv) {
    Options options{};

    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};

        if (arg == "--domain" && i + 1 < argc) {
            const std::string value{argv[++i]};
            try {
                std::size_t parsed_length = 0;
                const unsigned long parsed = std::stoul(value, &parsed_length);
                if (parsed_length != value.size() ||
                    parsed > std::numeric_limits<std::uint32_t>::max()) {
                    throw std::runtime_error("invalid --domain value");
                }
                options.domain_id = static_cast<std::uint32_t>(parsed);
            } catch (const std::exception&) {
                throw std::runtime_error("invalid --domain value");
            }
        } else if (arg == "--topic" && i + 1 < argc) {
            options.topic = argv[++i];
        } else if (arg == "--coordinates" && i + 1 < argc) {
            const std::string value{argv[++i]};
            if (value == "pico") {
                options.coordinates = pico_dds_bridge::CoordinateSystem::Pico;
            } else if (value == "robot") {
                options.coordinates = pico_dds_bridge::CoordinateSystem::Robot;
            } else if (value == "xrobot") {
                options.coordinates = pico_dds_bridge::CoordinateSystem::Xrobot;
            } else {
                throw std::runtime_error(
                    "invalid --coordinates value: " + value +
                    " (expected pico, robot, or xrobot)");
            }
        } else if (arg == "--require-shm") {
            options.require_shm = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: pico_dds_bridge [--domain N] [--topic NAME] [--coordinates pico|robot|xrobot] [--require-shm]\n"
                << "Default domain: 0\n"
                << "Default topic : pico/tracking\n"
                << "Coordinate output: pico (default), robot (X forward, Y left, Z up), or xrobot\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + std::string(arg));
        }
    }

    return options;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    try {
        const Options options = parse_options(argc, argv);

        pico_dds_bridge::DdsPublisher publisher(
            options.domain_id,
            options.topic);

        const bool shm_available = publisher.shared_memory_available();
        std::cerr << (shm_available
            ? "[dds] shared-memory/PSMX available\n"
            : "[dds] shared-memory/PSMX unavailable\n");
        if (options.require_shm && !shm_available) {
            return 3;
        }

        pico_dds_bridge::TrackingParser parser;
        pico_dds_bridge::XRoboToolkitClient client;

        if (!client.start()) {
            return 2;
        }

        std::cerr
            << "[bridge] running; DDS domain=" << options.domain_id
            << ", topic=" << options.topic << '\n';

        std::uint64_t sequence = 0;
        std::uint64_t frames_published = 0;
        std::uint64_t parse_failures = 0;
        std::uint64_t loan_failures = 0;
        std::uint64_t parse_time_us = 0;
        std::uint64_t fill_write_time_us = 0;
        std::chrono::steady_clock::time_point stats_time =
            std::chrono::steady_clock::now();
        std::uint64_t stats_frames = 0;

        while (g_running.load()) {
            pico_dds_bridge::RawFrameSlot* raw =
                client.wait_pop(std::chrono::milliseconds(100));
            if (!raw) {
                continue;
            }

            auto* sample = publisher.request_sample();
            if (sample == nullptr) {
                ++loan_failures;
                client.release(raw);
                continue;
            }

            const auto parse_start = std::chrono::steady_clock::now();
            const bool parsed = parser.parse_into(
                raw->data.get(), raw->length, raw->capacity,
                sequence++, raw->receive_timestamp_ns, *sample);
            const auto parse_end = std::chrono::steady_clock::now();
            client.release(raw);
            if (!parsed) {
                ++parse_failures;
                publisher.cancel(sample);
                continue;
            }

            pico_dds_bridge::convert_to_coordinates(*sample, options.coordinates);
            const auto write_start = std::chrono::steady_clock::now();
            publisher.publish(sample);
            const auto write_end = std::chrono::steady_clock::now();
            parse_time_us += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    parse_end - parse_start).count());
            fill_write_time_us += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    write_end - write_start).count());
            ++frames_published;
            ++stats_frames;

            const auto now = std::chrono::steady_clock::now();
            if (now - stats_time >= std::chrono::seconds(5)) {
                const double seconds = std::chrono::duration<double>(now - stats_time).count();
                std::cerr
                    << "[stats] json_capacity=" << pico_dds_bridge::kMaxJsonSize
                    << " parse_time_us="
                    << (stats_frames > 0 ? parse_time_us / stats_frames : 0)
                    << " dds_fill_write_time_us="
                    << (stats_frames > 0 ? fill_write_time_us / stats_frames : 0)
                    << " frame_hz=" << (seconds > 0.0 ? stats_frames / seconds : 0.0)
                    << " queue_drops=" << client.dropped_frames()
                    << " parse_failures=" << parse_failures
                    << " loan_failures=" << loan_failures
                    << " frames=" << frames_published
                    << " shm=" << (shm_available ? "true" : "false")
                    << '\n';
                stats_time = now;
                stats_frames = 0;
                parse_time_us = 0;
                fill_write_time_us = 0;
            }
        }

        client.stop();

        std::cerr
            << "[bridge] stopped; dropped callback frames="
            << client.dropped_frames() << '\n';

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "[bridge] fatal: " << e.what() << '\n';
        return 1;
    }
}

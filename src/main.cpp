#include "pico_dds_bridge/dds_publisher.hpp"
#include "pico_dds_bridge/coordinate_transform.hpp"
#include "pico_dds_bridge/parser.hpp"
#include "pico_dds_bridge/xrobotoolkit_client.hpp"

#include <atomic>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {

std::atomic_bool g_running{true};

void on_signal(int) {
    g_running.store(false);
}

struct Options {
    std::uint32_t domain_id{0};
    std::string topic{"pico/tracking"};
    bool robot_coordinates{false};
};

Options parse_options(const int argc, char** argv) {
    Options options{};

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};

        if (arg == "--domain" && i + 1 < argc) {
            const std::string_view value{argv[++i]};
            std::uint32_t parsed{};
            const auto [ptr, ec] = std::from_chars(
                value.data(), value.data() + value.size(), parsed);
            if (ec != std::errc{} || ptr != value.data() + value.size()) {
                throw std::runtime_error("invalid --domain value");
            }
            options.domain_id = parsed;
        } else if (arg == "--topic" && i + 1 < argc) {
            options.topic = argv[++i];
        } else if (arg == "--robot-coordinates") {
            options.robot_coordinates = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: pico_dds_bridge [--domain N] [--topic NAME] [--robot-coordinates]\n"
                << "Default domain: 0\n"
                << "Default topic : pico/tracking\n"
                << "Coordinate output: PICO (default), or robot (X forward, Y left, Z up)\n";
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

        pico_dds_bridge::TrackingParser parser;
        pico_dds_bridge::XRoboToolkitClient client(/*queue_capacity=*/8);

        if (!client.start()) {
            return 2;
        }

        std::cerr
            << "[bridge] running; DDS domain=" << options.domain_id
            << ", topic=" << options.topic << '\n';

        std::uint64_t sequence = 0;

        while (g_running.load()) {
            auto raw = client.wait_pop(std::chrono::milliseconds(100));
            if (!raw.has_value()) {
                continue;
            }

            auto frame = parser.parse(
                raw->json,
                sequence++,
                raw->receive_timestamp_ns);

            if (!frame.has_value()) {
                continue;
            }

            if (options.robot_coordinates) {
                pico_dds_bridge::convert_to_robot_coordinates(*frame);
            }

            publisher.publish(*frame);
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

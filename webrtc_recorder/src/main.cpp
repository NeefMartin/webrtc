// main.cpp — entry point for webrtc_recorder_node

#include "webrtc_recorder/webrtc_recorder_node.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace
{
// Global shutdown flag for signal handling
std::atomic<bool> g_shutdown_requested(false);

void signal_handler(int signum)
{
    (void)signum;
    g_shutdown_requested = true;
    rclcpp::shutdown();  // This will cause spin() to return
}

void print_usage(const char * prog)
{
    std::cout
        << "Usage: " << prog << " [options]\n\n"
        << "Options:\n"
        << "  --relay <url>     Relay WebSocket URL (default: ws://localhost:8765)\n"
        << "  --help            Show this help\n\n"
        << "Topics published:\n"
        << "  /camera/image/raw         sensor_msgs/msg/Image (BGR8)\n"
        << "  /audio/raw                audio_common_msgs/msg/AudioData (Opus)\n"
        << "  /ros_control              std_msgs/msg/String (JSON)\n\n"
        << "Example:\n"
        << "  " << prog << " --relay ws://192.168.1.100:8765\n";
}
}  // namespace

int main(int argc, char ** argv)
{
    std::string relay_url = "wss://localhost:8765";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--relay" && i + 1 < argc) {
            relay_url = argv[++i];
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<webrtc_recorder::WebRTCRecorderNode>();

    RCLCPP_INFO(node->get_logger(),
        "WebRTC recorder connecting to relay: %s", relay_url.c_str());

    // Register signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Create and run relay client in background thread
    webrtc_recorder::SignalingClient relay_client(node, relay_url);
    
    std::thread client_thread([&relay_client]() {
        relay_client.run([&]() -> bool {
            return g_shutdown_requested;
        });
    });

    // ROS 2 spin on main thread (will be interrupted by signal handler)
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down...");
    
    if (client_thread.joinable()) {
        client_thread.join();
    }

    RCLCPP_INFO(node->get_logger(),
        "Stats: %d video frames, %d audio frames, %d control messages.",
        node->video_count(), node->audio_count(), node->control_count());

    rclcpp::shutdown();
    return 0;
}

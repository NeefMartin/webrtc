#pragma once

// webrtc_recorder/webrtc_recorder_node.hpp
//
// WebRTC → ROS 2 topic bridge.
//
// Topics published
// ----------------
//   /camera/image/raw    sensor_msgs/msg/Image             BGR8 decoded frames
//   /audio/raw           audio_common_msgs/msg/AudioData   Opus passthrough
//   /ros_control         std_msgs/msg/String               Raw DataChannel JSON
//
// Video pipeline
// --------------
//   WebRTC H264 RTP  →  RtpDepacketizer (libdatachannel)
//       →  H264Decoder (FFmpeg libavcodec + libswscale)
//           →  sensor_msgs/Image  (encoding: bgr8)
//
// ROS 2 target: Jazzy Jalisco (Ubuntu 24.04)
// WebRTC library: libdatachannel >= 0.21
// Codec library:  FFmpeg (libavcodec, libavutil, libswscale)

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// FFmpeg C headers
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

// libdatachannel
#include <rtc/rtc.hpp>

// nlohmann JSON
#include <nlohmann/json.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <audio_common_msgs/msg/audio_data.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace webrtc_recorder
{

// ── Utility ───────────────────────────────────────────────────────────────────

/// Convert RTP presentation timestamp to nanoseconds.
int64_t pts_to_ns(int64_t pts, double time_base) noexcept;

/// Build a ROS Time stamp from nanoseconds.
builtin_interfaces::msg::Time ns_to_stamp(int64_t ns) noexcept;


// ── H264Decoder ───────────────────────────────────────────────────────────────

/// Wraps FFmpeg libavcodec to decode raw H264 NAL unit buffers into BGR8 frames.
///
/// Decoded frames are returned via a callback rather than copied into an
/// intermediate buffer, keeping allocations minimal.
///
/// Thread safety: not thread-safe — always call from the same thread.
class H264Decoder
{
public:
    H264Decoder();
    ~H264Decoder();

    // Non-copyable, non-movable (owns raw FFmpeg resource pointers)
    H264Decoder(const H264Decoder &)             = delete;
    H264Decoder & operator=(const H264Decoder &) = delete;

    /// Called once per decoded frame.
    /// @param bgr_data   Pointer to packed BGR8 pixel data
    /// @param width      Frame width in pixels
    /// @param height     Frame height in pixels
    /// @param step       Row stride in bytes (= width * 3 for packed BGR)
    using FrameCallback = std::function<
        void(const uint8_t * bgr_data, int width, int height, int step)>;

    /// Feed one H264 NAL unit (or a concatenation of NALs) to the decoder.
    /// Calls @p on_frame synchronously for each fully decoded output frame.
    /// One input packet may produce zero, one, or occasionally more frames.
    void decode(const std::vector<uint8_t> & data, const FrameCallback & on_frame);

    bool is_open() const noexcept { return codec_ctx_ != nullptr; }

private:
    /// (Re-)create the SwsContext if the frame dimensions changed.
    bool ensure_sws(int width, int height);

    AVCodecContext * codec_ctx_{nullptr};  ///< libavcodec decoder context
    AVFrame        * yuv_frame_{nullptr};  ///< decoded YUV420P output frame
    AVFrame        * bgr_frame_{nullptr};  ///< converted BGR24 frame
    AVPacket       * packet_{nullptr};     ///< reusable input packet
    SwsContext     * sws_ctx_{nullptr};    ///< YUV→BGR colour-space converter

    // Cached source dimensions to detect resolution changes
    int sws_src_width_{0};
    int sws_src_height_{0};
};


// ── WebRTCRecorderNode ────────────────────────────────────────────────────────

/// ROS 2 node — receives WebRTC media + DataChannel and publishes ROS topics.
/// Does not manage bagging; use `ros2 bag record` separately.
class WebRTCRecorderNode : public rclcpp::Node
{
public:
    explicit WebRTCRecorderNode();
    ~WebRTCRecorderNode() override = default;

    // ── Publishing interface called by StreamerSession ────────────────────────

    /// Decode an H264 NAL unit buffer and publish as sensor_msgs/Image (bgr8).
    void publish_video(const std::vector<uint8_t> & data, int64_t stamp_ns);

    /// Publish a raw Opus audio frame as AudioData.
    void publish_audio(const std::vector<uint8_t> & data, int64_t stamp_ns);

    /// Publish a raw DataChannel JSON string as std_msgs/String.
    void publish_control(const std::string & json_str);

    // Statistics
    int video_count()   const noexcept { return video_count_.load(); }
    int audio_count()   const noexcept { return audio_count_.load(); }
    int control_count() const noexcept { return control_count_.load(); }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           image_pub_;
    rclcpp::Publisher<audio_common_msgs::msg::AudioData>::SharedPtr audio_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             control_pub_;

    // H264Decoder is not thread-safe; guard it so that if libdatachannel
    // ever dispatches onFrame callbacks from different threads the decoder
    // state remains consistent.
    std::unique_ptr<H264Decoder> decoder_;
    std::mutex                   decoder_mutex_;

    std::atomic<int> video_count_{0};
    std::atomic<int> audio_count_{0};
    std::atomic<int> control_count_{0};
};


// ── StreamerSession ───────────────────────────────────────────────────────────

/// Manages one WebRTC peer connection from a single streamer.html tab.
class StreamerSession : public std::enable_shared_from_this<StreamerSession>
{
public:
    StreamerSession(
        std::weak_ptr<rtc::WebSocket>       ws,
        std::shared_ptr<WebRTCRecorderNode> node);

    void handle_offer(const std::string & sdp, const std::string & type);
    void handle_candidate(const std::string & candidate, const std::string & mid);

private:
    void setup_video_track(std::shared_ptr<rtc::Track> track);
    void setup_audio_track(std::shared_ptr<rtc::Track> track);
    void setup_data_channel(std::shared_ptr<rtc::DataChannel> channel);

    std::weak_ptr<rtc::WebSocket>         ws_;
    std::shared_ptr<WebRTCRecorderNode>   node_;
    std::shared_ptr<rtc::PeerConnection>  pc_;

    std::shared_ptr<rtc::Track>           video_track_;
    std::shared_ptr<rtc::Track>           audio_track_;
    std::shared_ptr<rtc::RtpDepacketizer> video_depacketizer_;
    std::shared_ptr<rtc::RtpDepacketizer> audio_depacketizer_;
    std::shared_ptr<rtc::DataChannel>     data_channel_;
};


// ── SignalingServer ───────────────────────────────────────────────────────────

/// WebSocket signaling server compatible with streamer.html.
class SignalingServer
{
public:
    SignalingServer(
        std::shared_ptr<WebRTCRecorderNode> node,
        uint16_t                             port);

    void run();

private:
    void on_client(std::shared_ptr<rtc::WebSocket> ws);

    std::shared_ptr<WebRTCRecorderNode>   node_;
    uint16_t                              port_;
    std::unique_ptr<rtc::WebSocketServer> server_;
};

// ── SignalingClient ───────────────────────────────────────────────────────────

/// WebSocket client connecting to an external relay server (e.g., Python server.py).
/// Registers as a "streamer" and relays WebRTC signaling messages.
class SignalingClient
{
public:
    SignalingClient(
        std::shared_ptr<WebRTCRecorderNode> node,
        const std::string &                  relay_url);
    
    ~SignalingClient();

    /// Start the client (blocking until shutdown is requested).
    void run(std::function<bool()> shutdown_check = nullptr);

private:
    void on_relay_message(const std::variant<rtc::binary, std::string> & data);
    void send_to_relay(const nlohmann::json & msg);

    std::shared_ptr<WebRTCRecorderNode>   node_;
    std::string                            relay_url_;
    std::shared_ptr<rtc::WebSocket>       relay_ws_;
    std::shared_ptr<StreamerSession>      session_;  // Persists across connection lifetime
};

}  // namespace webrtc_recorder
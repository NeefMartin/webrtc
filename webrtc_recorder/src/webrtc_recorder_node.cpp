// webrtc_recorder/webrtc_recorder_node.cpp
//
// Implementation of H264Decoder, WebRTCRecorderNode, StreamerSession,
// and SignalingServer.
//
// Video path:
//   H264 NAL bytes  →  avcodec_send_packet / avcodec_receive_frame
//       →  sws_scale (YUV420P → BGR24)
//           →  sensor_msgs/Image (bgr8)  →  /camera/image/raw

#include "webrtc_recorder/webrtc_recorder_node.hpp"

#include <cstring>
#include <filesystem>
#include <future>
#include <stdexcept>

using json = nlohmann::json;

static constexpr double H264_TIME_BASE = 1.0 / 90000.0;
static constexpr double OPUS_TIME_BASE = 1.0 / 48000.0;

namespace webrtc_recorder
{

// ── Utility ───────────────────────────────────────────────────────────────────

int64_t pts_to_ns(int64_t pts, double time_base) noexcept
{
    if (pts <= 0) return 0;
    return static_cast<int64_t>(static_cast<double>(pts) * time_base * 1e9);
}

builtin_interfaces::msg::Time ns_to_stamp(int64_t ns) noexcept
{
    builtin_interfaces::msg::Time t;
    t.sec    = static_cast<int32_t>(ns / 1'000'000'000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
    return t;
}


static std::vector<uint8_t> copy_rtc_binary(const rtc::binary & frame)
{
    std::vector<uint8_t> data(frame.size());
    if (!data.empty()) {
        std::memcpy(data.data(), frame.data(), frame.size());
    }
    return data;
}


static void process_signaling_message(
    const json & msg,
    const std::shared_ptr<StreamerSession> & session,
    const rclcpp::Logger & logger)
{
    const std::string type = msg.value("type", std::string{});
    if (type == "register") {
        return;
    } else if (type == "offer") {
        const auto & sdp = msg.at("sdp");
        session->handle_offer(
            sdp.value("sdp",  std::string{}),
            sdp.value("type", std::string{"offer"}));
    } else if (type == "candidate") {
        const auto & cand = msg.at("candidate");
        if (cand.is_object()) {
            session->handle_candidate(
                cand.value("candidate", std::string{}),
                cand.value("sdpMid",    std::string{"0"}));
        }
    } else {
        RCLCPP_DEBUG(logger, "Unsupported signaling type: %s", type.c_str());
    }
}


// ── H264Decoder ───────────────────────────────────────────────────────────────

H264Decoder::H264Decoder()
{
    // Find and open the H264 decoder
    const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        throw std::runtime_error("H264Decoder: avcodec_find_decoder(H264) failed — "
                                 "is FFmpeg built with H264 support?");
    }

    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) {
        throw std::runtime_error("H264Decoder: avcodec_alloc_context3 failed");
    }

    // Allow the decoder to receive incomplete frames (common with WebRTC streams)
    codec_ctx_->flags2 |= AV_CODEC_FLAG2_CHUNKS;

    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
        avcodec_free_context(&codec_ctx_);
        throw std::runtime_error("H264Decoder: avcodec_open2 failed");
    }

    // Allocate reusable frame and packet buffers
    yuv_frame_ = av_frame_alloc();
    bgr_frame_ = av_frame_alloc();
    packet_    = av_packet_alloc();

    if (!yuv_frame_ || !bgr_frame_ || !packet_) {
        // Cleanup handled in destructor
        throw std::runtime_error("H264Decoder: av_frame_alloc / av_packet_alloc failed");
    }
}

H264Decoder::~H264Decoder()
{
    if (sws_ctx_)   { sws_freeContext(sws_ctx_);  sws_ctx_   = nullptr; }
    if (packet_)    { av_packet_free(&packet_);   packet_    = nullptr; }
    if (yuv_frame_) { av_frame_free(&yuv_frame_); yuv_frame_ = nullptr; }
    if (bgr_frame_) { av_frame_free(&bgr_frame_); bgr_frame_ = nullptr; }
    if (codec_ctx_) { avcodec_free_context(&codec_ctx_); codec_ctx_ = nullptr; }
}

bool H264Decoder::ensure_sws(int width, int height)
{
    if (sws_ctx_ && sws_src_width_ == width && sws_src_height_ == height) {
        return true;  // Already configured for this resolution
    }

    // Free old context if dimensions changed (e.g. resolution switch mid-stream)
    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }

    sws_ctx_ = sws_getContext(
        width, height, AV_PIX_FMT_YUV420P,   // source: decoded YUV
        width, height, AV_PIX_FMT_BGR24,      // dest:   packed BGR24
        SWS_BILINEAR,
        nullptr, nullptr, nullptr);

    if (!sws_ctx_) {
        return false;
    }

    // Allocate the BGR output frame buffer
    av_frame_unref(bgr_frame_);
    bgr_frame_->format = AV_PIX_FMT_BGR24;
    bgr_frame_->width  = width;
    bgr_frame_->height = height;
    if (av_frame_get_buffer(bgr_frame_, 1) < 0) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
        return false;
    }

    sws_src_width_  = width;
    sws_src_height_ = height;
    return true;
}

void H264Decoder::decode(
    const std::vector<uint8_t> & data,
    const FrameCallback          & on_frame)
{
    if (!codec_ctx_) return;

    // Fill the AVPacket with the incoming NAL data — av_packet_from_data
    // would take ownership, so we use the manual approach to avoid a copy
    // of the vector's buffer being freed by FFmpeg.
    packet_->data = const_cast<uint8_t *>(data.data());
    packet_->size = static_cast<int>(data.size());

    const int send_rc = avcodec_send_packet(codec_ctx_, packet_);
    if (send_rc < 0 && send_rc != AVERROR(EAGAIN)) {
        // Non-fatal — malformed or incomplete NAL; skip silently
        return;
    }

    // Receive all available decoded frames (typically 0 or 1, rarely more)
    while (true) {
        const int recv_rc = avcodec_receive_frame(codec_ctx_, yuv_frame_);
        if (recv_rc == AVERROR(EAGAIN) || recv_rc == AVERROR_EOF) {
            break;   // No more frames available from this packet
        }
        if (recv_rc < 0) {
            break;   // Decode error — skip frame
        }

        // Convert YUV420P → BGR24 via libswscale
        if (!ensure_sws(yuv_frame_->width, yuv_frame_->height)) {
            av_frame_unref(yuv_frame_);
            continue;
        }

        sws_scale(
            sws_ctx_,
            yuv_frame_->data,     yuv_frame_->linesize,
            0, yuv_frame_->height,
            bgr_frame_->data,     bgr_frame_->linesize);

        // Deliver the BGR frame to the caller
        // step = bytes per row = width * 3 channels for packed BGR24
        const int step = bgr_frame_->linesize[0];
        on_frame(bgr_frame_->data[0],
                 bgr_frame_->width,
                 bgr_frame_->height,
                 step);

        av_frame_unref(yuv_frame_);
    }
}


// ── WebRTCRecorderNode ────────────────────────────────────────────────────────

WebRTCRecorderNode::WebRTCRecorderNode()
: rclcpp::Node("webrtc_recorder")
{
    // QoS: best effort, depth 1 — drop old frames rather than building a backlog
    auto media_qos   = rclcpp::QoS(1).best_effort();
    // QoS: reliable for control messages — ordered delivery, never drop
    auto control_qos = rclcpp::QoS(10).reliable();

    image_pub_   = create_publisher<sensor_msgs::msg::Image>(
        "/camera/image/raw", media_qos);

    audio_pub_   = create_publisher<audio_common_msgs::msg::AudioData>(
        "/audio/raw", media_qos);

    control_pub_ = create_publisher<std_msgs::msg::String>(
        "/ros_control", control_qos);

    // Initialise the H264 decoder — throws on failure (missing FFmpeg codec)
    try {
        decoder_ = std::make_unique<H264Decoder>();
        RCLCPP_INFO(get_logger(), "H264 decoder initialised (FFmpeg libavcodec)");
    } catch (const std::exception & e) {
        RCLCPP_FATAL(get_logger(), "Failed to initialise H264 decoder: %s", e.what());
        throw;
    }

    RCLCPP_INFO(get_logger(), "Publishers created:");
    RCLCPP_INFO(get_logger(), "  /camera/image/raw  (sensor_msgs/Image bgr8, best_effort)");
    RCLCPP_INFO(get_logger(), "  /audio/raw         (audio_common_msgs/AudioData, best_effort)");
    RCLCPP_INFO(get_logger(), "  /ros_control       (std_msgs/String, reliable)");
    RCLCPP_INFO(get_logger(), "Use 'ros2 bag record' to record any of these topics.");
}

void WebRTCRecorderNode::publish_video(
    const std::vector<uint8_t> & data,
    int64_t                       stamp_ns)
{
    const builtin_interfaces::msg::Time stamp =
        ns_to_stamp(stamp_ns > 0 ? stamp_ns : now().nanoseconds());

    // Decode under the mutex — H264Decoder is not thread-safe
    std::lock_guard<std::mutex> lock(decoder_mutex_);

    decoder_->decode(data,
        [this, &stamp](const uint8_t * bgr_data, int width, int height, int step)
        {
            sensor_msgs::msg::Image msg;
            msg.header.stamp    = stamp;
            msg.header.frame_id = "camera";
            msg.width           = static_cast<uint32_t>(width);
            msg.height          = static_cast<uint32_t>(height);
            msg.encoding        = "bgr8";
            msg.is_bigendian    = false;
            msg.step            = static_cast<uint32_t>(step);

            // Copy the BGR pixel data into the message
            const std::size_t data_size = static_cast<std::size_t>(step) *
                                          static_cast<std::size_t>(height);
            msg.data.assign(bgr_data, bgr_data + data_size);

            image_pub_->publish(msg);

            const int n = ++video_count_;
            if (n % 30 == 0) {
                RCLCPP_DEBUG(get_logger(),
                    "Published %d decoded video frames (%dx%d)", n, width, height);
            }
        });
}

void WebRTCRecorderNode::publish_audio(
    const std::vector<uint8_t> & data,
    int64_t                       stamp_ns)
{
    (void)stamp_ns;  // AudioData has no header in audio_common_msgs
    audio_common_msgs::msg::AudioData msg;
    msg.data = data;
    audio_pub_->publish(msg);
    ++audio_count_;
}

void WebRTCRecorderNode::publish_control(const std::string & json_str)
{
    std_msgs::msg::String msg;
    msg.data = json_str;
    control_pub_->publish(msg);

    const int n = ++control_count_;
    RCLCPP_DEBUG(get_logger(),
        "Published control message #%d: %s", n, json_str.c_str());
}


// ── StreamerSession ───────────────────────────────────────────────────────────

StreamerSession::StreamerSession(
    std::weak_ptr<rtc::WebSocket>       ws,
    std::shared_ptr<WebRTCRecorderNode> node)
: ws_(ws), node_(node)
{
    rtc::Configuration config;
    config.iceServers.emplace_back("stun:stun.l.google.com:19302");
    pc_ = std::make_shared<rtc::PeerConnection>(config);

    pc_->onLocalDescription([this](rtc::Description desc) {
        const json answer = {
            {"type", "answer"},
            {"sdp", {
                {"type", std::string(desc.typeString())},
                {"sdp",  std::string(desc)}
            }}
        };
        if (auto ws = ws_.lock()) ws->send(answer.dump());
    });

    pc_->onLocalCandidate([this](rtc::Candidate cand) {
        const json msg = {
            {"type", "candidate"},
            {"candidate", {
                {"candidate",     std::string(cand)},
                {"sdpMid",        cand.mid()},
                {"sdpMLineIndex", 0}
            }}
        };
        if (auto ws = ws_.lock()) ws->send(msg.dump());
    });

    pc_->onTrack([this](std::shared_ptr<rtc::Track> track) {
        const std::string sdp_frag = track->description().generateSdp();
        if (sdp_frag.find("video") != std::string::npos) {
            RCLCPP_INFO(node_->get_logger(), "Video track received");
            setup_video_track(track);
        } else {
            RCLCPP_INFO(node_->get_logger(), "Audio track received");
            setup_audio_track(track);
        }
    });

    pc_->onDataChannel([this](std::shared_ptr<rtc::DataChannel> channel) {
        RCLCPP_INFO(node_->get_logger(),
            "DataChannel received: %s", channel->label().c_str());
        setup_data_channel(channel);
    });

    pc_->onStateChange([this](rtc::PeerConnection::State state) {
        const char* state_str = "unknown";
        switch (state) {
            case rtc::PeerConnection::State::New:         state_str = "new"; break;
            case rtc::PeerConnection::State::Connecting:  state_str = "connecting"; break;
            case rtc::PeerConnection::State::Connected:   state_str = "connected"; break;
            case rtc::PeerConnection::State::Disconnected: state_str = "disconnected"; break;
            case rtc::PeerConnection::State::Failed:      state_str = "failed"; break;
            case rtc::PeerConnection::State::Closed:      state_str = "closed"; break;
        }
        RCLCPP_INFO(node_->get_logger(),
            "Peer connection state: %s", state_str);
    });
}

void StreamerSession::handle_offer(
    const std::string & sdp,
    const std::string & type)
{
    const bool has_video = sdp.find("m=video") != std::string::npos;
    RCLCPP_INFO(node_->get_logger(),
        "Offer received (%s)", has_video ? "video + audio" : "audio only");
    pc_->setRemoteDescription(rtc::Description(sdp, type));
}

void StreamerSession::handle_candidate(
    const std::string & candidate,
    const std::string & mid)
{
    try {
        pc_->addRemoteCandidate(rtc::Candidate(candidate, mid));
    } catch (const std::exception & e) {
        RCLCPP_DEBUG(node_->get_logger(),
            "ICE candidate error (non-fatal): %s", e.what());
    }
}

void StreamerSession::setup_video_track(std::shared_ptr<rtc::Track> track)
{
    // Attach H264 depacketizer to the track
    auto depacketizer = std::make_shared<rtc::H264RtpDepacketizer>();
    track->setMediaHandler(depacketizer);

    // Set up frame callback for depacketized H264 data
    track->onFrame([this](rtc::binary frame, rtc::FrameInfo info) {
        const int64_t stamp_ns = pts_to_ns(
            static_cast<int64_t>(info.timestamp), H264_TIME_BASE);
        const auto data = copy_rtc_binary(frame);
        node_->publish_video(data, stamp_ns);
    });

    video_track_ = std::move(track);
}

void StreamerSession::setup_audio_track(std::shared_ptr<rtc::Track> track)
{
    // Attach Opus depacketizer to the track
    auto depacketizer = std::make_shared<rtc::OpusRtpDepacketizer>();
    track->setMediaHandler(depacketizer);

    // Set up frame callback for depacketized Opus data
    track->onFrame([this](rtc::binary frame, rtc::FrameInfo info) {
        const int64_t stamp_ns = pts_to_ns(
            static_cast<int64_t>(info.timestamp), OPUS_TIME_BASE);
        const auto data = copy_rtc_binary(frame);
        node_->publish_audio(data, stamp_ns);
    });

    audio_track_ = std::move(track);
}

void StreamerSession::setup_data_channel(
    std::shared_ptr<rtc::DataChannel> channel)
{
    channel->onOpen([this, channel]() {
        RCLCPP_INFO(node_->get_logger(),
            "DataChannel '%s' open", channel->label().c_str());
    });

    channel->onMessage([this](std::variant<rtc::binary, std::string> data) {
        if (!std::holds_alternative<std::string>(data)) return;
        node_->publish_control(std::get<std::string>(data));
    });

    channel->onClosed([this, channel]() {
        RCLCPP_INFO(node_->get_logger(),
            "DataChannel '%s' closed", channel->label().c_str());
    });

    data_channel_ = std::move(channel);
}


// ── SignalingServer ───────────────────────────────────────────────────────────

SignalingServer::SignalingServer(
    std::shared_ptr<WebRTCRecorderNode> node,
    uint16_t                             port)
: node_(node), port_(port)
{}

void SignalingServer::run()
{
    rtc::WebSocketServer::Configuration config;
    config.port = port_;

    try {
        const auto exe_dir =
            std::filesystem::canonical("/proc/self/exe").parent_path();
        const auto cert = exe_dir / "cert.pem";
        const auto key  = exe_dir / "key.pem";
        if (std::filesystem::exists(cert) && std::filesystem::exists(key)) {
            config.enableTls          = true;
            config.certificatePemFile = cert.string();
            config.keyPemFile         = key.string();
            RCLCPP_INFO(node_->get_logger(), "TLS enabled (wss://)");
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "cert.pem / key.pem not found — using plain ws://");
            RCLCPP_WARN(node_->get_logger(),
                "Browsers will block camera/mic on non-localhost origins.");
        }
    } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
            "TLS cert detection failed: %s", e.what());
    }

    server_ = std::make_unique<rtc::WebSocketServer>(config);
    server_->onClient([this](std::shared_ptr<rtc::WebSocket> ws) {
        on_client(ws);
    });

    const std::string proto = config.enableTls ? "wss" : "ws";
    RCLCPP_INFO(node_->get_logger(),
        "Signaling server ready on %s://0.0.0.0:%d", proto.c_str(), port_);
    RCLCPP_INFO(node_->get_logger(),
        "Open streamer.html on PC-2 and point it to %s://<VM-IP>:%d",
        proto.c_str(), port_);

    std::promise<void> exit_promise;
    exit_promise.get_future().wait();
}

void SignalingServer::on_client(std::shared_ptr<rtc::WebSocket> ws)
{
    RCLCPP_INFO(node_->get_logger(), "Streamer connected");

    auto session = std::make_shared<StreamerSession>(ws, node_);

    ws->onMessage([this, session](std::variant<rtc::binary, std::string> data)
    {
        if (!std::holds_alternative<std::string>(data)) return;
        try {
            const auto msg = json::parse(std::get<std::string>(data));
            process_signaling_message(msg, session, node_->get_logger());
        }
        catch (const json::exception & e) {
            RCLCPP_WARN(node_->get_logger(), "JSON error: %s", e.what());
        }
        catch (const std::exception & e) {
            RCLCPP_WARN(node_->get_logger(), "Message error: %s", e.what());
        }
    });

    ws->onClosed([this]() {
        RCLCPP_INFO(node_->get_logger(), "Streamer disconnected");
    });

    ws->onError([this](const std::string & err) {
        RCLCPP_WARN(node_->get_logger(), "WebSocket error: %s", err.c_str());
    });
}


// ─────────────────────────────────────────────────────────────────────────────
// SignalingClient — WebSocket client connecting to relay server
// ─────────────────────────────────────────────────────────────────────────────

SignalingClient::SignalingClient(
    std::shared_ptr<WebRTCRecorderNode> node,
    const std::string &                  relay_url)
: node_(node), relay_url_(relay_url)
{
}

SignalingClient::~SignalingClient()
{
    if (relay_ws_) {
        relay_ws_->close();
    }
}

void SignalingClient::send_to_relay(const nlohmann::json & msg)
{
    if (relay_ws_ && relay_ws_->isOpen()) {
        relay_ws_->send(msg.dump());
    }
}

void SignalingClient::on_relay_message(const std::variant<rtc::binary, std::string> & data)
{
    try {
        std::string msg_str;
        if (std::holds_alternative<std::string>(data)) {
            msg_str = std::get<std::string>(data);
        } else {
            const auto & bin = std::get<rtc::binary>(data);
            msg_str = std::string(reinterpret_cast<const char*>(bin.data()), bin.size());
        }

        const json msg = json::parse(msg_str);
        RCLCPP_INFO(node_->get_logger(), "Offer received from relay");
        if (session_) {
            process_signaling_message(msg, session_, node_->get_logger());
        }
    } catch (const json::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "JSON error: %s", e.what());
    } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "Message error: %s", e.what());
    }
}

void SignalingClient::run(std::function<bool()> shutdown_check)
{
    RCLCPP_INFO(node_->get_logger(),
        "Connecting to relay server: %s", relay_url_.c_str());

    try {
        rtc::WebSocket::Configuration ws_config;
        ws_config.disableTlsVerification = true;  // Accept self-signed certificates
        relay_ws_ = std::make_shared<rtc::WebSocket>(ws_config);
        
        bool on_open_called = false;
        bool on_closed_called = false;
        bool on_error_called = false;

        relay_ws_->onOpen([this, &on_open_called]() {
            on_open_called = true;
            RCLCPP_INFO(node_->get_logger(), "✓ Connected to relay server");
            
            // Register as streamer
            json register_msg = {
                {"type", "register"},
                {"role", "viewer"}
            };
            send_to_relay(register_msg);
            RCLCPP_INFO(node_->get_logger(), "✓ Registered as viewer");
            
            // Create session as member variable (persists across connection lifetime)
            this->session_ = std::make_shared<StreamerSession>(relay_ws_, node_);
        });

        relay_ws_->onMessage([this](const std::variant<rtc::binary, std::string> & data) {
            if (!this->session_) return;  // Session not yet created
            if (!std::holds_alternative<std::string>(data)) return;
            try {
                const auto msg  = json::parse(std::get<std::string>(data));
                const auto type = msg.value("type", std::string{});

                if (type == "register") {
                    // streamer.html compatibility — no action needed here
                } else if (type == "offer") {
                    const auto & sdp = msg.at("sdp");
                    this->session_->handle_offer(
                        sdp.value("sdp",  std::string{}),
                        sdp.value("type", std::string{"offer"}));
                } else if (type == "candidate") {
                    const auto & cand = msg.at("candidate");
                    if (cand.is_object()) {
                        this->session_->handle_candidate(
                            cand.value("candidate", std::string{}),
                            cand.value("sdpMid",    std::string{"0"}));
                    }
                }
            }
            catch (const json::exception & e) {
                RCLCPP_WARN(node_->get_logger(), "JSON error: %s", e.what());
            }
            catch (const std::exception & e) {
                RCLCPP_WARN(node_->get_logger(), "Message error: %s", e.what());
            }
        });

        relay_ws_->onClosed([this, &on_closed_called]() {
            on_closed_called = true;
            RCLCPP_WARN(node_->get_logger(), "⚠ Disconnected from relay");
        });

        relay_ws_->onError([this, &on_error_called](const std::string & err) {
            on_error_called = true;
            RCLCPP_ERROR(node_->get_logger(), 
                "✗ Relay WebSocket error: %s", err.c_str());
        });

        RCLCPP_INFO(node_->get_logger(), 
            "→ Opening WebSocket to relay: %s", relay_url_.c_str());
        relay_ws_->open(relay_url_);

        // Wait for connection to establish
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        if (!on_open_called && !on_error_called) {
            RCLCPP_WARN(node_->get_logger(),
                "⚠ WebSocket still connecting after 2 seconds...");
        }
        
        if (on_error_called) {
            RCLCPP_ERROR(node_->get_logger(),
                "Connection failed. Ensure server.py is running on %s", relay_url_.c_str());
            return;
        }

        // Poll for shutdown
        bool warned = false;
        auto start_time = std::chrono::steady_clock::now();
        
        while (true) {
            if (shutdown_check && shutdown_check()) {
                RCLCPP_INFO(node_->get_logger(), "Shutdown requested");
                break;
            }

            // Warn if not connected after 5 seconds
            if (!warned && on_closed_called) {
                auto elapsed = std::chrono::steady_clock::now() - start_time;
                if (elapsed > std::chrono::seconds(5)) {
                    RCLCPP_WARN(node_->get_logger(),
                        "⚠ Connection dropped to relay. Check:\n"
                        "  1. Is Python server still running? (ps aux | grep server.py)\n"
                        "  2. Is relay URL correct? (%s)\n"
                        "  3. Any firewall issues?", relay_url_.c_str());
                    warned = true;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (relay_ws_) {
            relay_ws_->close();
        }
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(),
            "SignalingClient exception: %s", e.what());
    }
}

}  // namespace webrtc_recorder

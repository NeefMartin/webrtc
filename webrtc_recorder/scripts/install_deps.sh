#!/usr/bin/env bash
# install_deps.sh
# Installs all system and ROS 2 Jazzy dependencies for webrtc_recorder.
# Called automatically by CMakeLists.txt at configure time.
# Safe to run multiple times.

set -euo pipefail

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()    { echo -e "${GREEN}[deps]${NC} $*"; }
warning() { echo -e "${YELLOW}[deps]${NC} $*"; }
error()   { echo -e "${RED}[deps]${NC}   $*" >&2; }

# ── Check apt is available ────────────────────────────────────────────────────
if ! command -v apt-get &>/dev/null; then
    warning "apt-get not found — skipping automatic install."
    warning "Install manually: libssl-dev nlohmann-json3-dev libopencv-dev cmake git"
    exit 0
fi

SUDO=""
if [ "$EUID" -ne 0 ]; then
    command -v sudo &>/dev/null && SUDO="sudo" || { error "Need root or sudo."; exit 1; }
fi

info "Updating apt..."
$SUDO apt-get update -qq

# ── Build tools ───────────────────────────────────────────────────────────────
info "Installing build tools..."
$SUDO apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config ninja-build

# ── SSL (for WSS in libdatachannel) ──────────────────────────────────────────
$SUDO apt-get install -y --no-install-recommends libssl-dev

# ── nlohmann JSON ─────────────────────────────────────────────────────────────
$SUDO apt-get install -y --no-install-recommends nlohmann-json3-dev

# ── libdatachannel ────────────────────────────────────────────────────────────
if apt-cache show libdatachannel-dev &>/dev/null 2>&1; then
    info "Installing libdatachannel from apt..."
    $SUDO apt-get install -y --no-install-recommends libdatachannel-dev
else
    warning "libdatachannel-dev not in apt — CMake will fetch it via FetchContent."
    # Install build deps for the FetchContent build
    $SUDO apt-get install -y --no-install-recommends \
        libsrtp2-dev libusrsctp-dev libplog-dev 2>/dev/null || true
fi

# ── ROS 2 Jazzy packages ──────────────────────────────────────────────────────
ROS_DISTRO="jazzy"
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    warning "ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}."
    warning "Install ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html"
    exit 0
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

info "Installing ROS 2 ${ROS_DISTRO} packages..."
$SUDO apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-rclcpp" \
    "ros-${ROS_DISTRO}-sensor-msgs" \
    "ros-${ROS_DISTRO}-std-msgs" \
    "ros-${ROS_DISTRO}-builtin-interfaces" \
    "ros-${ROS_DISTRO}-rcutils" \
    "ros-${ROS_DISTRO}-audio-common-msgs" \
    "ros-${ROS_DISTRO}-rosbag2" \
    "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
    "ros-${ROS_DISTRO}-ros2launch" 2>/dev/null || {
        warning "Some ROS packages could not be installed — may need manual install."
    }

# ── FFmpeg (H264 decoder)
info "Installing FFmpeg development libraries..."
$SUDO apt-get install -y --no-install-recommends \
    libavcodec-dev \
    libavutil-dev \
    libswscale-dev

# ── Python deps for server.py ─────────────────────────────────────────────────
info "Installing Python dependencies for server.py..."
pip3 install --quiet websockets cryptography 2>/dev/null || \
    warning "pip install failed — install websockets manually: pip3 install websockets"

info "All dependencies installed."
info ""
info "Build:"
info "  cd ~/ros2_ws && colcon build --packages-select webrtc_recorder robot_stream"
info "  source install/setup.bash"
info ""
info "Launch:"
info "  ros2 launch robot_stream robot_stream.launch.py"

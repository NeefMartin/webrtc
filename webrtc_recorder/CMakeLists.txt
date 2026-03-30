cmake_minimum_required(VERSION 3.22)
project(webrtc_recorder VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FETCH_LIBDATACHANNEL
    "Fetch and build libdatachannel via FetchContent if not found in system"
    ON)

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(audio_common_msgs  REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcutils             REQUIRED)
find_package(cv_bridge          REQUIRED)
find_package(OpenCV             REQUIRED)
find_package(ament_index_cpp REQUIRED)
# ── FFmpeg (libavcodec + libavutil + libswscale) ──────────────────────────────
# Used by H264Decoder to decode H264 NAL units into BGR8 frames.
# PkgConfig is the standard way to find FFmpeg on Ubuntu 24.04.
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC  REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL   REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(SWSCALE  REQUIRED IMPORTED_TARGET libswscale)

# ── nlohmann/json ─────────────────────────────────────────────────────────────
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.3
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# ── libdatachannel ────────────────────────────────────────────────────────────
find_package(LibDataChannel QUIET)
if(LibDataChannel_FOUND)
    message(STATUS "Using system libdatachannel")
    set(_ldc_target LibDataChannel::LibDataChannel)
elseif(FETCH_LIBDATACHANNEL)
    message(STATUS "Fetching libdatachannel from GitHub...")
    include(FetchContent)
    FetchContent_Declare(libdatachannel
        GIT_REPOSITORY     https://github.com/paullouisageneau/libdatachannel.git
        GIT_TAG            v0.21.2
        GIT_SHALLOW        TRUE
        GIT_SUBMODULES_RECURSE TRUE)
    set(NO_EXAMPLES ON  CACHE BOOL "" FORCE)
    set(NO_TESTS    ON  CACHE BOOL "" FORCE)
    set(NO_MEDIA    OFF CACHE BOOL "" FORCE)
    set(USE_GNUTLS  OFF CACHE BOOL "" FORCE)
    set(USE_NICE    OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(libdatachannel)
    set(_ldc_target datachannel)
else()
    message(FATAL_ERROR
        "libdatachannel not found. Install via apt or set FETCH_LIBDATACHANNEL=ON")
endif()

# ── webrtc_recorder_lib ───────────────────────────────────────────────────────
add_library(webrtc_recorder_lib STATIC
    src/webrtc_recorder_node.cpp)

target_include_directories(webrtc_recorder_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(webrtc_recorder_lib
    rclcpp sensor_msgs audio_common_msgs std_msgs builtin_interfaces rcutils ament_index_cpp)

target_link_libraries(webrtc_recorder_lib
        ${_ldc_target}
        nlohmann_json::nlohmann_json
        # FFmpeg targets (via PkgConfig IMPORTED_TARGET)
        PkgConfig::AVCODEC
        PkgConfig::AVUTIL
        PkgConfig::SWSCALE)

# ── webrtc_recorder_node ──────────────────────────────────────────────────────
add_executable(webrtc_recorder_node src/main.cpp)

ament_target_dependencies(webrtc_recorder_node rclcpp)

target_link_libraries(webrtc_recorder_node webrtc_recorder_lib)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
    webrtc_recorder_lib
    webrtc_recorder_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(PROGRAMS scripts/install_deps.sh
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
    rclcpp
    sensor_msgs
    audio_common_msgs
    std_msgs
    builtin_interfaces
    rcutils
)

ament_package()
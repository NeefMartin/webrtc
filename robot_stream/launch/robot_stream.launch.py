"""
robot_stream.launch.py
======================
Launches the complete robot stream system on the VM:

  1. server.py                — HTTP file server + WS signaling relay
  2. webrtc_recorder_node     — WebRTC relay client + ROS 2 topic publisher

Together they bridge a WebRTC stream from streamer.html (PC-2 browser)
into three ROS 2 topics:

  /camera/image/raw          sensor_msgs/msg/Image            (BGR8 decoded)
  /audio/raw                 audio_common_msgs/msg/AudioData  (Opus passthrough)
  /ros_control               std_msgs/msg/String              (JSON DataChannel)

The C++ recorder node connects to the Python relay server and registers as a
"streamer" client, allowing it to receive WebRTC offers and handle video/audio.

Recording
---------
  Topics are published but NOT recorded here.
  To record, run in a separate terminal:
    ros2 bag record /camera/image/raw /audio/raw /ros_control

Usage
-----
  ros2 launch robot_stream robot_stream.launch.py
  ros2 launch robot_stream robot_stream.launch.py ws_port:=8765 http_port:=8080
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Declare arguments ─────────────────────────────────────────────────────
    ws_port_arg = DeclareLaunchArgument(
        "ws_port",
        default_value="8765",
        description="WebSocket signaling port used by server.py relay and "
                    "webrtc_recorder_node client connection",
    )

    http_port_arg = DeclareLaunchArgument(
        "http_port",
        default_value="8080",
        description="HTTPS port for the static file server (streamer/viewer HTML)",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 log level: debug / info / warn / error",
    )

    # ── Locate server.py inside the installed robot_stream package ─────────────
    robot_stream_share = get_package_share_directory("robot_stream")

    # server.py is installed to lib/robot_stream/ by CMakeLists.txt
    robot_stream_lib = os.path.join(
        os.path.dirname(robot_stream_share),  # share/../
        "..",                                  # up to install/
        "lib",
        "robot_stream",
        "server.py",
    )
    server_py = os.path.realpath(robot_stream_lib)

    # Static directory is installed to share/robot_stream/static/
    static_dir = os.path.join(robot_stream_share, "static")

    # ── 1. server.py — HTTPS + WSS signaling server ───────────────────────────
    # Launched as a plain process (not a ROS node) so it can serve HTTPS
    # on its own asyncio loop without interference from rclpy.
    #
    # server.py reads ws_port and http_port from environment variables
    # ROBOT_STREAM_WS_PORT and ROBOT_STREAM_HTTP_PORT so they can be
    # overridden from the launch file without modifying the script.
    signaling_server = ExecuteProcess(
        cmd=[
            "python3", server_py,
        ],
        additional_env={
            "ROBOT_STREAM_WS_PORT":   LaunchConfiguration("ws_port"),
            "ROBOT_STREAM_HTTP_PORT": LaunchConfiguration("http_port"),
            "ROBOT_STREAM_STATIC_DIR": static_dir,
        },
        name="signaling_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    # ── 2. webrtc_recorder_node — WebRTC relay client + ROS 2 publisher ──────
    # Connects to the relay server as a WebSocket client with role "streamer"
    recorder_node = Node(
        package="webrtc_recorder",
        executable="webrtc_recorder_node",
        name="webrtc_recorder",
        output="screen",
        arguments=[
            "--relay", ["wss://localhost:", LaunchConfiguration("ws_port")],
        ],
        parameters=[
            {"use_sim_time": False},
        ],
        ros_arguments=[
            "--log-level", LaunchConfiguration("log_level"),
        ],
        respawn=True,
        respawn_delay=3.0,
    )

    # ── Startup info ──────────────────────────────────────────────────────────
    startup_info = LogInfo(
        msg=[
            "\n",
            "=" * 60, "\n",
            "  robot_stream system starting\n",
            "  WS relay port  : ", LaunchConfiguration("ws_port"),   "\n",
            "  HTTP port      : ", LaunchConfiguration("http_port"), "\n",
            "  Static dir     : ", static_dir, "\n",
            "\n",
            "  Topics published by webrtc_recorder_node:\n",
            "    /camera/image/raw          (sensor_msgs/Image BGR8)\n",
            "    /audio/raw                 (audio_common_msgs/AudioData)\n",
            "    /ros_control               (std_msgs/String JSON)\n",
            "\n",
            "  To record all topics, run in a separate terminal:\n",
            "    ros2 bag record /camera/image/raw /audio/raw /ros_control\n",
            "=" * 60, "\n",
        ]
    )

    return LaunchDescription(
        [
            ws_port_arg,
            http_port_arg,
            log_level_arg,
            startup_info,
            signaling_server,
            # Delay the recorder node by 3 seconds to give server.py time to bind
            TimerAction(period=3.0, actions=[recorder_node]),
        ]
    )

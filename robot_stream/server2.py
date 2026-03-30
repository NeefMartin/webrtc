#!/usr/bin/env python3
"""
server.py — WebRTC Signaling + HTTPS file server
Runs on the VM (Ubuntu 24.04, ROS 2 Jazzy).

Serves streamer.html / viewer.html over HTTPS and relays WebRTC
signaling messages (offer / answer / ICE candidates) between
the browser on PC-2 and the webrtc_recorder_node running on the VM.

Ports
-----
  8080  HTTPS — serves static/
  8765  WSS   — WebRTC signaling

Signaling model (fan-out per viewer)
-------------------------------------
  Each viewer gets a unique viewer_id. When a viewer registers, the
  streamer is notified with viewer_joined{viewer_id}. The streamer
  creates a dedicated RTCPeerConnection for that viewer and sends an
  offer tagged with the viewer_id. All subsequent messages (answer,
  candidate) carry the viewer_id so the server can route them to the
  correct single peer instead of broadcasting to all.

Certificate
-----------
  Generated automatically on first run using openssl or the
  'cryptography' Python package as fallback. Stored as cert.pem
  and key.pem next to this file. Both files include a SAN entry
  for the detected LAN IP so browsers accept the certificate.
"""

import asyncio
import itertools
import os
import json
import logging
import socket
import ssl
import subprocess
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

import websockets

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("robot_stream")

BASE_DIR   = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
CERT_FILE  = BASE_DIR / "cert.pem"
KEY_FILE   = BASE_DIR / "key.pem"

# ── Connected clients ─────────────────────────────────────────────────────────
# viewers:   dict mapping viewer_id -> websocket
# streamers: set of websocket connections (usually just one)
viewers:   dict = {}
streamers: set  = set()

# Monotonically increasing counter for viewer IDs
_viewer_id_counter = itertools.count(1)


def next_viewer_id() -> str:
    return f"viewer_{next(_viewer_id_counter)}"


# ── Networking helpers ────────────────────────────────────────────────────────

def get_local_ip() -> str:
    """Return the LAN IPv4 address of this machine."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


# ── TLS certificate ───────────────────────────────────────────────────────────

def generate_cert(local_ip: str) -> None:
    """
    Generate a self-signed TLS certificate with SAN entries for:
      - DNS: localhost
      - IP:  127.0.0.1
      - IP:  <local_ip>

    Uses openssl if available, falls back to the cryptography package.
    Skips generation if the existing cert already covers local_ip.
    """
    if CERT_FILE.exists() and KEY_FILE.exists():
        try:
            result = subprocess.run(
                ["openssl", "x509", "-in", str(CERT_FILE), "-text", "-noout"],
                capture_output=True, text=True, check=True,
            )
            if local_ip in result.stdout:
                log.info("Existing certificate covers %s — reusing.", local_ip)
                return
            log.info("IP changed or cert missing SAN — regenerating.")
        except (subprocess.CalledProcessError, FileNotFoundError):
            pass

    log.info("Generating self-signed TLS certificate (SAN: localhost, %s)…", local_ip)

    # ── openssl ───────────────────────────────────────────────────────────────
    cfg_path = BASE_DIR / "_san.cfg"
    cfg_path.write_text(
        "[req]\ndistinguished_name=req\n"
        "[san]\n"
        f"subjectAltName=DNS:localhost,IP:127.0.0.1,IP:{local_ip}\n"
    )
    try:
        subprocess.run(
            [
                "openssl", "req", "-x509", "-newkey", "rsa:2048",
                "-keyout", str(KEY_FILE), "-out", str(CERT_FILE),
                "-days", "365", "-nodes",
                "-subj", "/CN=robot-stream",
                "-extensions", "san",
                "-config", str(cfg_path),
            ],
            check=True, capture_output=True,
        )
        cfg_path.unlink(missing_ok=True)
        log.info("Certificate generated via openssl.")
        return
    except (subprocess.CalledProcessError, FileNotFoundError):
        cfg_path.unlink(missing_ok=True)

    # ── cryptography package fallback ─────────────────────────────────────────
    try:
        import datetime
        import ipaddress

        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import rsa
        from cryptography.x509.oid import NameOID

        key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "robot-stream")])
        cert = (
            x509.CertificateBuilder()
            .subject_name(name)
            .issuer_name(name)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(datetime.datetime.utcnow())
            .not_valid_after(
                datetime.datetime.utcnow() + datetime.timedelta(days=365)
            )
            .add_extension(
                x509.SubjectAlternativeName(
                    [
                        x509.DNSName("localhost"),
                        x509.IPAddress(ipaddress.IPv4Address("127.0.0.1")),
                        x509.IPAddress(ipaddress.IPv4Address(local_ip)),
                    ]
                ),
                critical=False,
            )
            .sign(key, hashes.SHA256())
        )
        KEY_FILE.write_bytes(
            key.private_bytes(
                serialization.Encoding.PEM,
                serialization.PrivateFormat.TraditionalOpenSSL,
                serialization.NoEncryption(),
            )
        )
        CERT_FILE.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
        log.info("Certificate generated via cryptography package.")
    except ImportError:
        log.error("Cannot generate certificate — install openssl or run:")
        log.error("  pip install cryptography")
        raise SystemExit(1)


def make_ssl_context() -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=str(CERT_FILE), keyfile=str(KEY_FILE))
    return ctx


# ── WebSocket signaling ───────────────────────────────────────────────────────

async def handle(websocket: websockets.WebSocketServerProtocol) -> None:
    """
    Handle one WebSocket connection from either a streamer or a viewer.

    Routing rules
    -------------
    Viewer registration:
      viewer → server:   { type: "register", role: "viewer" }
      server → streamer: { type: "viewer_joined", viewer_id: "viewer_N" }

    Streamer registration:
      streamer → server:  { type: "register", role: "streamer" }
      server → viewer(s): { type: "streamer_joined" }   (one per existing viewer)

    Streamer → viewer (tagged with viewer_id):
      { type: "offer",     viewer_id: "viewer_N", sdp: ... }
      { type: "candidate", viewer_id: "viewer_N", candidate: ... }
      Server strips viewer_id before forwarding.

    Viewer → streamer (tagged with viewer_id so streamer knows which PC):
      { type: "answer",    viewer_id: "viewer_N", sdp: ... }
      { type: "candidate", viewer_id: "viewer_N", candidate: ... }
      Server forwards the full message so the streamer can route it to
      the correct RTCPeerConnection.
    """
    role: str | None      = None
    my_viewer_id: str | None = None

    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                log.warning("Received invalid JSON — ignoring.")
                continue

            msg_type: str = msg.get("type", "")

            # ── Registration ──────────────────────────────────────────────────
            if msg_type == "register":
                role = msg.get("role")

                if role == "viewer":
                    my_viewer_id = next_viewer_id()
                    viewers[my_viewer_id] = websocket
                    log.info("Viewer registered: %s (total: %d)",
                             my_viewer_id, len(viewers))
                    # Tell every active streamer that a new viewer is ready
                    await broadcast(streamers, {
                        "type":      "viewer_joined",
                        "viewer_id": my_viewer_id,
                    })

                elif role == "streamer":
                    streamers.add(websocket)
                    log.info("Streamer registered (total: %d)", len(streamers))
                    # Tell the new streamer about every viewer already waiting
                    for vid in list(viewers.keys()):
                        await websocket.send(json.dumps({
                            "type":      "viewer_joined",
                            "viewer_id": vid,
                        }))

            # ── Streamer → specific viewer ────────────────────────────────────
            elif msg_type in ("offer", "candidate") and role == "streamer":
                target_id = msg.get("viewer_id")
                target_ws = viewers.get(target_id)
                if target_ws:
                    # Strip viewer_id before forwarding — viewers don't need it
                    fwd = {k: v for k, v in msg.items() if k != "viewer_id"}
                    try:
                        await target_ws.send(json.dumps(fwd))
                        log.debug("Relayed %s → %s", msg_type, target_id)
                    except Exception as exc:
                        log.warning("Failed to relay %s to %s: %s",
                                    msg_type, target_id, exc)
                else:
                    log.warning("Viewer %s not found for %s", target_id, msg_type)

            # ── Viewer → streamer (keep viewer_id so streamer can route) ──────
            elif msg_type in ("answer", "candidate") and role == "viewer":
                # Attach our viewer_id if the viewer didn't include it
                msg.setdefault("viewer_id", my_viewer_id)
                await broadcast(streamers, msg)
                log.debug("Relayed %s from %s → streamers", msg_type, my_viewer_id)

    except websockets.exceptions.ConnectionClosedOK:
        pass
    except websockets.exceptions.ConnectionClosedError as exc:
        log.warning("Connection closed with error: %s", exc)
    finally:
        if role == "viewer" and my_viewer_id:
            viewers.pop(my_viewer_id, None)
            log.info("Viewer disconnected: %s (total: %d)",
                     my_viewer_id, len(viewers))
            # Tell every streamer this viewer is gone
            await broadcast(streamers, {
                "type":      "viewer_left",
                "viewer_id": my_viewer_id,
            })
        elif role == "streamer":
            streamers.discard(websocket)
            log.info("Streamer disconnected (total: %d)", len(streamers))
            # Tell every remaining viewer the streamer left
            await broadcast(set(viewers.values()), {"type": "streamer_left"})


async def broadcast(targets, msg: dict) -> None:
    """Send msg to every websocket in targets (set or iterable)."""
    targets = list(targets)
    if not targets:
        return
    data = json.dumps(msg)
    await asyncio.gather(
        *[t.send(data) for t in targets],
        return_exceptions=True,
    )


# ── HTTPS static file server ──────────────────────────────────────────────────

class StaticHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, fmt, *args):  # silence per-request logs
        pass


def run_https_thread(port: int, ssl_ctx: ssl.SSLContext) -> None:
    if not STATIC_DIR.exists():
        log.error("Static directory not found: %s", STATIC_DIR)
        log.error("Make sure static/ is next to server.py")
        return
    server = HTTPServer(("0.0.0.0", port), StaticHandler)
    server.socket = ssl_ctx.wrap_socket(server.socket, server_side=True)
    log.info("HTTPS server → https://0.0.0.0:%d  (serving %s)", port, STATIC_DIR)
    server.serve_forever()


# ── Entry point ───────────────────────────────────────────────────────────────

async def main(
    ws_port:   int = int(os.environ.get("ROBOT_STREAM_WS_PORT",   "8765")),
    http_port: int = int(os.environ.get("ROBOT_STREAM_HTTP_PORT", "8080")),
) -> None:
    # Allow launch file to override the static directory
    global STATIC_DIR
    env_static = os.environ.get("ROBOT_STREAM_STATIC_DIR", "")
    if env_static:
        STATIC_DIR = Path(env_static)
    local_ip = get_local_ip()
    generate_cert(local_ip)
    ssl_ctx = make_ssl_context()

    # Start HTTPS file server in a daemon thread
    http_thread = threading.Thread(
        target=run_https_thread, args=(http_port, ssl_ctx), daemon=True
    )
    http_thread.start()

    log.info("=" * 62)
    log.info("VM LAN IP detected : %s", local_ip)
    log.info("")
    log.info("  PC-2 browser (streamer):")
    log.info("    https://%s:%d/streamer.html", local_ip, http_port)
    log.info("")
    log.info("  VM browser (viewer / monitor):")
    log.info("    https://localhost:%d/viewer.html", http_port)
    log.info("")
    log.info("  Certificate warning → Advanced → Proceed (both browsers)")
    log.info("  Also accept the WS cert once at: https://%s:%d", local_ip, ws_port)
    log.info("=" * 62)

    async with websockets.serve(handle, "0.0.0.0", ws_port, ssl=ssl_ctx):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())

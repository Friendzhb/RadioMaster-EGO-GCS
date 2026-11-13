#!/usr/bin/env python3
"""ROS1 -> bounded UDP display telemetry. No socket input, command publishing or FCU access."""

import argparse
import math
import secrets
import socket
import struct
import threading
import time
import zlib

HEADER = struct.Struct("<4sBBHIIHHIIQ")
MAX_DATAGRAM = 1200
FRAGMENT_BYTES = MAX_DATAGRAM - HEADER.size
ODOMETRY = 1
CLOUD = 2


def datagrams(kind, session, sequence, payload, stamp_ms):
    limit = 40 if kind == ODOMETRY else 8000 * 16
    if kind not in (ODOMETRY, CLOUD) or not payload or len(payload) > limit:
        raise ValueError("unsupported/oversized telemetry")
    if (kind == ODOMETRY and len(payload) != 40) or (kind == CLOUD and len(payload) % 16):
        raise ValueError("invalid payload layout")
    fragments = (len(payload) + FRAGMENT_BYTES - 1) // FRAGMENT_BYTES
    crc = zlib.crc32(payload)
    for index in range(fragments):
        offset = index * FRAGMENT_BYTES
        yield HEADER.pack(
            b"EGUD", 1, kind, 0, session, sequence, index, fragments,
            len(payload), crc, stamp_ms,
        ) + payload[offset:offset + FRAGMENT_BYTES]


def cloud_payload(message, max_points):
    """Evenly sample rows with padding; accept FLOAT32/FLOAT64 coordinates."""
    width, height = int(message.width), int(message.height)
    step, row_step = int(message.point_step), int(message.row_step)
    if not 1 <= max_points <= 8000 or width <= 0 or height <= 0 or step <= 0:
        raise ValueError("invalid cloud dimensions")
    if row_step < width * step or len(message.data) < row_step * height:
        raise ValueError("truncated cloud")
    fields = {f.name: f for f in message.fields}
    endian = ">" if message.is_bigendian else "<"

    def reader(name, required=True):
        f = fields.get(name)
        if f is None and not required:
            return None
        formats = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}
        allowed = (7, 8) if required else formats
        if f is None or f.datatype not in allowed or f.count < 1:
            raise ValueError("unsupported coordinate/intensity field: " + name)
        unpack = struct.Struct(endian + formats[f.datatype])
        if f.offset < 0 or f.offset + unpack.size > step:
            raise ValueError("invalid field offset")
        return f.offset, unpack

    xyz = [reader(n) for n in ("x", "y", "z")]
    intensity_name = "intensity" if "intensity" in fields else "reflectivity"
    intensity = reader(intensity_name, False)
    total = width * height
    keep = min(total, max_points)
    out = bytearray()
    for sample in range(keep):
        i = sample * (total - 1) // max(1, keep - 1)
        base = i // width * row_step + i % width * step
        point = [u.unpack_from(message.data, base + offset)[0] for offset, u in xyz]
        if not all(math.isfinite(v) and abs(v) <= 3.4e38 for v in point):
            continue
        value = intensity[1].unpack_from(message.data, base + intensity[0])[0] if intensity else 0.5
        if not math.isfinite(value):
            value = 0.5
        if value > 1.5:
            value /= 255.0
        out.extend(struct.pack("<4f", *point, min(1.0, max(0.0, value))))
    return bytes(out)


class Bridge:
    def __init__(self, args, rospy):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2

        self.args, self.rospy = args, rospy
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)
        self.socket.settimeout(0.01)
        self.target = (socket.gethostbyname(args.target), args.port)
        self.session = secrets.randbits(32)
        self.sequence = {ODOMETRY: 0, CLOUD: 0}
        self.condition = threading.Condition()
        self.pending_cloud = None
        self.running = True
        self.last_odom = 0.0
        self.sent = self.dropped = 0
        self.odom_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.subscribers = [
            rospy.Subscriber(args.odom_topic, Odometry, self.odometry, queue_size=1, tcp_nodelay=True),
            rospy.Subscriber(args.cloud_topic, PointCloud2, self.cloud, queue_size=1, buff_size=16 * 1024 * 1024),
        ]
        self.thread = threading.Thread(target=self.cloud_worker, name="gcs-udp-cloud", daemon=True)
        self.thread.start()

    def send(self, kind, payload):
        if not payload:
            return
        # Reserve a sequence under a tiny lock; odometry never waits for a fragmented cloud.
        with self.send_lock:
            self.sequence[kind] = (self.sequence[kind] + 1) & 0xffffffff
            sequence = self.sequence[kind]
        try:
            for packet in datagrams(kind, self.session, sequence, payload, time.monotonic_ns() // 1_000_000):
                self.socket.sendto(packet, self.target)
            self.sent += 1
        except OSError:
            self.dropped += 1

    def odometry(self, message):
        now = time.monotonic()
        with self.odom_lock:
            if now - self.last_odom < 1.0 / self.args.odom_hz:
                return
            self.last_odom = now
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
        values = (p.x, p.y, p.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z)
        if all(math.isfinite(x) and abs(x) <= 3.4e38 for x in values):
            self.send(ODOMETRY, struct.pack("<10f", *values))

    def cloud(self, message):
        with self.condition:
            self.pending_cloud = message
            self.condition.notify()

    def cloud_worker(self):
        while self.running:
            with self.condition:
                self.condition.wait_for(lambda: self.pending_cloud is not None or not self.running)
                if not self.running:
                    return
                message, self.pending_cloud = self.pending_cloud, None
            started = time.monotonic()
            try:
                self.send(CLOUD, cloud_payload(message, self.args.max_points))
            except (ValueError, struct.error) as error:
                self.rospy.logwarn_throttle(5.0, "GCS UDP cloud rejected: %s", error)
            time.sleep(max(0.0, 1.0 / self.args.cloud_hz - (time.monotonic() - started)))

    def close(self):
        self.running = False
        with self.condition:
            self.condition.notify_all()
        self.socket.close()


def main():
    import rospy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True, help="AX12 Wi-Fi IP")
    parser.add_argument("--port", type=int, default=14560)
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--odom-hz", type=float, default=50.0)
    parser.add_argument("--cloud-hz", type=float, default=10.0)
    parser.add_argument("--max-points", type=int, default=8000)
    args = parser.parse_args(rospy.myargv()[1:])
    if not (1024 <= args.port <= 65535 and args.port != 14550 and 0 < args.odom_hz <= 100 and 0 < args.cloud_hz <= 15 and 1 <= args.max_points <= 8000):
        parser.error("invalid port, rate or point limit")
    rospy.init_node("ego_gcs_udp_telemetry", anonymous=True)
    bridge = Bridge(args, rospy)
    rospy.on_shutdown(bridge.close)
    rospy.loginfo("GCS UDP display telemetry -> %s:%d; commands remain on TCPROS/rosbridge", args.target, args.port)
    rospy.spin()


if __name__ == "__main__":
    main()
